// simple_device_identifier.cpp
// Compile with: g++ -o simple_device_identifier simple_device_identifier.cpp -pthread

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
#include <algorithm>

// Structure to hold device information
struct DeviceInfo {
    std::string path;      // Device path (e.g., /dev/ttyUSB0)
    std::string type;      // Device type (e.g., TEENSY1, TEENSY2)
    int fd;                // File descriptor
    bool identified;       // Whether the device has been identified

    DeviceInfo() : fd(-1), identified(false) {}
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

// Function to listen for device identifiers
void listenForDevices() {
    // Get all TTY devices
    std::vector<std::string> ttyDevices = findTtyUsbDevices();
    printf("Found %zu potential TTY devices\n", ttyDevices.size());

    // Open each device and listen for identifiers
    for (const auto& ttyDevice : ttyDevices) {
        // Check if we already identified this device
        bool alreadyIdentified = false;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            for (const auto& pair : g_devices) {
                if (pair.second.path == ttyDevice && pair.second.identified) {
                    alreadyIdentified = true;
                    break;
                }
            }
        }

        if (alreadyIdentified) {
            continue;
        }

        printf("Checking %s for identifier...\n", ttyDevice.c_str());

        int fd = openSerialPort(ttyDevice.c_str());
        if (fd == -1) {
            continue;
        }

        // Listen for identifier
        char buffer[256] = {0};
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';

            // Print raw response
            printf("Raw response from %s: '%s'\n", ttyDevice.c_str(), buffer);

            // Check for known identifiers
            std::string deviceType;
            if (strstr(buffer, "TEENSY1_ID")) {
                deviceType = "TEENSY1";
            } else if (strstr(buffer, "TEENSY2_ID")) {
                deviceType = "TEENSY2";
            }

            if (!deviceType.empty()) {
                printf("Identified %s as %s\n", ttyDevice.c_str(), deviceType.c_str());

                // Send acknowledgment
                const char* ack = "ACK\n";
                write(fd, ack, strlen(ack));
                printf("Sent ACK to %s\n", ttyDevice.c_str());

                // Store device info
                std::lock_guard<std::mutex> lock(g_mutex);
                DeviceInfo device;
                device.path = ttyDevice;
                device.type = deviceType;
                device.fd = fd;
                device.identified = true;
                g_devices[deviceType] = device;

                printf("Added %s to device map\n", deviceType.c_str());
            } else {
                // Not one of our devices or not sending identifier
                close(fd);
            }
        } else {
            // No data received
            close(fd);
        }
    }
}

// Function to get a device file descriptor by type
int getDeviceFd(const std::string& deviceType) {
    std::lock_guard<std::mutex> lock(g_mutex);

    auto it = g_devices.find(deviceType);
    if (it != g_devices.end() && it->second.identified) {
        return it->second.fd;
    }

    return -1;
}

// Function to get a device path by type
std::string getDevicePath(const std::string& deviceType) {
    std::lock_guard<std::mutex> lock(g_mutex);

    auto it = g_devices.find(deviceType);
    if (it != g_devices.end() && it->second.identified) {
        return it->second.path;
    }

    return "";
}

// Function to list all identified devices
void listIdentifiedDevices() {
    std::lock_guard<std::mutex> lock(g_mutex);

    printf("Identified devices:\n");
    for (const auto& pair : g_devices) {
        const DeviceInfo& device = pair.second;
        if (device.identified) {
            printf("  %s: %s (fd=%d)\n", device.type.c_str(), device.path.c_str(), device.fd);
        }
    }
}

// Main function
int main() {
    printf("Simple device identifier starting...\n");

    // Main loop - scan for devices until all are found
    while (g_running) {
        // Listen for device identifiers
        listenForDevices();

        // List identified devices
        listIdentifiedDevices();

        // Check if we found all expected devices
        bool allFound = false;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            allFound = g_devices.find("TEENSY1") != g_devices.end() && 
                       g_devices.find("TEENSY2") != g_devices.end();
        }

        if (allFound) {
            printf("All devices identified!\n");

            // Get file descriptors for specific devices
            int teensy1_fd = getDeviceFd("TEENSY1");
            int teensy2_fd = getDeviceFd("TEENSY2");

            printf("TEENSY1 is available at %s with fd=%d\n", 
                   getDevicePath("TEENSY1").c_str(), teensy1_fd);
            printf("TEENSY2 is available at %s with fd=%d\n", 
                   getDevicePath("TEENSY2").c_str(), teensy2_fd);

            // In a real application, you would now pass these file descriptors
            // to your packet processing code

            // For this example, we'll just read and discard some data
            printf("\nReading some data from each device...\n");

            for (int i = 0; i < 5; i++) {
                // Read from TEENSY1
                if (teensy1_fd != -1) {
                    char buffer[256] = {0};
                    int bytesRead = read(teensy1_fd, buffer, sizeof(buffer));

                    if (bytesRead > 0) {
                        printf("TEENSY1 data (%d bytes): ", bytesRead);
                        for (int j = 0; j < bytesRead; j++) {
                            printf("%02X ", (unsigned char)buffer[j]);
                        }
                        printf("\n");
                    }
                }

                // Read from TEENSY2
                if (teensy2_fd != -1) {
                    char buffer[256] = {0};
                    int bytesRead = read(teensy2_fd, buffer, sizeof(buffer));

                    if (bytesRead > 0) {
                        printf("TEENSY2 data (%d bytes): ", bytesRead);
                        for (int j = 0; j < bytesRead; j++) {
                            printf("%02X ", (unsigned char)buffer[j]);
                        }
                        printf("\n");
                    }
                }

                sleep(1);
            }

            break;  // Exit the loop once we've found all devices
        }

        // Wait before scanning again
        sleep(1);
    }

    // Clean up
    std::lock_guard<std::mutex> lock(g_mutex);
    for (auto& pair : g_devices) {
        DeviceInfo& device = pair.second;
        if (device.fd != -1) {
            close(device.fd);
            device.fd = -1;
        }
    }

    printf("Device identifier complete\n");
    return 0;
}
