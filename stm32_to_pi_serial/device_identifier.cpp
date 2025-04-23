// device_identifier.cpp
// Compile with: g++ -o device_identifier device_identifier.cpp -pthread

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>
#include <pthread.h>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <chrono>
#include <thread>
#include <signal.h>

// Structure to hold device information
struct DeviceInfo {
    std::string path;      // Device path (e.g., /dev/ttyUSB0)
    std::string type;      // Device type (e.g., TEENSY1, TEENSY2)
    int fd;                // File descriptor
    bool connected;        // Connection status

    DeviceInfo() : fd(-1), connected(false) {}
};

// Global variables
std::map<std::string, DeviceInfo> g_devices;
std::mutex g_mutex;
bool g_running = true;

// Function to open and configure a serial port
int openSerialPort(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("Error getting serial settings");
        close(fd);
        return -1;
    }

    // Configure baud rate
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Configure 8N1 (8 data bits, No parity, 1 stop bit)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // Set read timeout
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("Error applying serial settings");
        close(fd);
        return -1;
    }

    // Clear any pending data
    tcflush(fd, TCIOFLUSH);

    return fd;
}

// Function to identify a device by sending a query and checking response
std::string identifyDevice(int fd) {
    // Clear any pending data
    tcflush(fd, TCIOFLUSH);

    // Send identification request
    const char* query = "IDENTIFY\n";
    write(fd, query, strlen(query));

    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    char buffer[64] = {0};
    int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

    if (bytesRead > 0) {
        buffer[bytesRead] = '\0';

        // Remove any newlines or carriage returns
        char* newline = strchr(buffer, '\n');
        if (newline) *newline = '\0';

        newline = strchr(buffer, '\r');
        if (newline) *newline = '\0';

        printf("Device responded: '%s'\n", buffer);

        // Check if response is one of our expected device types
        if (strstr(buffer, "TEENSY1") || strstr(buffer, "TEENSY2")) {
            return buffer;
        }
    }

    return "";
}

// Function to find all TTY USB devices
std::vector<std::string> findTtyUsbDevices() {
    std::vector<std::string> devices;
    DIR* dir = opendir("/dev/");

    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL) {
            std::string name = entry->d_name;
            if (name.find("ttyUSB") == 0 || name.find("ttyACM") == 0) {
                devices.push_back("/dev/" + name);
            }
        }
        closedir(dir);
    }

    return devices;
}

// Function to scan for and identify devices
void scanForDevices() {
    std::vector<std::string> ttyDevices = findTtyUsbDevices();
    std::vector<std::string> foundDevices;

    printf("Found %zu potential TTY devices\n", ttyDevices.size());

    // Try to identify each device
    for (const auto& ttyDevice : ttyDevices) {
        printf("Checking %s...\n", ttyDevice.c_str());

        int fd = openSerialPort(ttyDevice.c_str());
        if (fd != -1) {
            std::string deviceType = identifyDevice(fd);

            if (!deviceType.empty()) {
                printf("Identified %s as %s\n", ttyDevice.c_str(), deviceType.c_str());

                // Lock before modifying the devices map
                std::lock_guard<std::mutex> lock(g_mutex);

                // Check if we already have this device type
                auto it = g_devices.find(deviceType);
                if (it != g_devices.end()) {
                    // We already have this device type
                    DeviceInfo& device = it->second;

                    // If it's already connected, skip
                    if (device.connected) {
                        printf("%s is already connected at %s\n", 
                               deviceType.c_str(), device.path.c_str());
                        close(fd);
                    }
                    else {
                        // Update the device info
                        device.path = ttyDevice;
                        device.fd = fd;
                        device.connected = true;
                        printf("%s reconnected at %s\n", deviceType.c_str(), ttyDevice.c_str());
                    }
                }
                else {
                    // New device type
                    DeviceInfo device;
                    device.type = deviceType;
                    device.path = ttyDevice;
                    device.fd = fd;
                    device.connected = true;
                    g_devices[deviceType] = device;

                    printf("Added new device %s\n", deviceType.c_str());
                }

                foundDevices.push_back(deviceType);
            }
            else {
                // Not one of our devices
                close(fd);
            }
        }
    }

    // Check for disconnected devices
    std::lock_guard<std::mutex> lock(g_mutex);
    for (auto& pair : g_devices) {
        const std::string& deviceType = pair.first;
        DeviceInfo& device = pair.second;

        if (device.connected && 
            std::find(foundDevices.begin(), foundDevices.end(), deviceType) == foundDevices.end()) {
            printf("%s disconnected\n", deviceType.c_str());
            device.connected = false;
            close(device.fd);
            device.fd = -1;
        }
    }
}

// Background thread for device scanning
void* scanThread(void* arg) {
    while (g_running) {
        scanForDevices();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    return NULL;
}

// Signal handler for clean shutdown
void signalHandler(int signum) {
    printf("Caught signal %d, cleaning up...\n", signum);
    g_running = false;

    // Close all device file descriptors
    std::lock_guard<std::mutex> lock(g_mutex);
    for (auto& pair : g_devices) {
        DeviceInfo& device = pair.second;
        if (device.fd != -1) {
            close(device.fd);
            device.fd = -1;
        }
    }

    exit(signum);
}

// Function to get a device file descriptor by type
int getDeviceFd(const std::string& deviceType) {
    std::lock_guard<std::mutex> lock(g_mutex);

    auto it = g_devices.find(deviceType);
    if (it != g_devices.end() && it->second.connected) {
        return it->second.fd;
    }

    return -1;
}

// Function to get a device path by type
std::string getDevicePath(const std::string& deviceType) {
    std::lock_guard<std::mutex> lock(g_mutex);

    auto it = g_devices.find(deviceType);
    if (it != g_devices.end() && it->second.connected) {
        return it->second.path;
    }

    return "";
}

// Function to list all connected devices
void listConnectedDevices() {
    std::lock_guard<std::mutex> lock(g_mutex);

    printf("Connected devices:\n");
    for (const auto& pair : g_devices) {
        const DeviceInfo& device = pair.second;
        if (device.connected) {
            printf("  %s: %s (fd=%d)\n", device.type.c_str(), device.path.c_str(), device.fd);
        }
    }
}

// Initialize the device scanner in the background
pthread_t startDeviceScanner() {
    pthread_t thread;
    pthread_create(&thread, NULL, scanThread, NULL);

    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    return thread;
}

// Stop the device scanner
void stopDeviceScanner(pthread_t thread) {
    g_running = false;
    pthread_join(thread, NULL);
}

// Example of how to use this with your existing code
int main() {
    printf("Device identifier starting...\n");

    // Start the background scanner
    pthread_t scannerThread = startDeviceScanner();

    // Wait for devices to be discovered
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Main processing loop
    while (g_running) {
        // List all connected devices
        listConnectedDevices();

        // Example: Get file descriptors for specific devices
        int teensy1_fd = getDeviceFd("TEENSY1");
        int teensy2_fd = getDeviceFd("TEENSY2");

        if (teensy1_fd != -1) {
            printf("TEENSY1 is available with fd=%d\n", teensy1_fd);

            // Here you would call your existing packet processing code
            // processPacketsFromDevice(teensy1_fd);
        }

        if (teensy2_fd != -1) {
            printf("TEENSY2 is available with fd=%d\n", teensy2_fd);

            // Here you would call your existing packet processing code
            // processPacketsFromDevice(teensy2_fd);
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Clean up
    stopDeviceScanner(scannerThread);

    return 0;
}
