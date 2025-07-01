// integrated_device_receiver.cpp
// Compile with: g++ -o integrated_device_receiver integrated_device_receiver.cpp -pthread

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
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>
#include <signal.h>

// Constants
#define BUFFER_SIZE 256
#define PACKET_SIZE 7
#define START_MARKER 0x3C
#define END_MARKER 0x3E
#define MAX_DEVICES 2

float 

// Device information structure
struct DeviceInfo {
    std::string path;
    std::string type;
    int fd;
    pthread_t thread;
    bool running;
    bool connected;
    std::chrono::steady_clock::time_point lastActivity;
    std::mutex mutex;  // For thread-safe access to non-atomic members

    DeviceInfo() : fd(-1), running(false), connected(false) {}

    // Non-copyable
    DeviceInfo(const DeviceInfo&) = delete;
    DeviceInfo& operator=(const DeviceInfo&) = delete;
};

// Global variables
std::map<std::string, std::unique_ptr<DeviceInfo>> devices;
std::mutex device_mutex;
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

// Function to check if a device is sending an identifier
bool checkForIdentifier(int fd, std::string& deviceType) {
    // Read data
    char buffer[256] = {0};
    int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

    if (bytesRead > 0) {
        buffer[bytesRead] = '\0';

        // Print raw data for debugging
        printf("Received %d bytes: '", bytesRead);
        for (int i = 0; i < bytesRead; i++) {
            if (buffer[i] >= 32 && buffer[i] <= 126) {
                printf("%c", buffer[i]);
            } else {
                printf("\\x%02X", (unsigned char)buffer[i]);
            }
        }
        printf("'\n");

        // Check for identifiers
        if (strstr(buffer, "TEENSY1_ID")) {
            deviceType = "TEENSY1";
            return true;
        } else if (strstr(buffer, "TEENSY2_ID")) {
            deviceType = "TEENSY2";
            return true;
        } else if (strstr(buffer, "STM32_ID")) {
            deviceType = "STM32";
            return true;
        }
    }

    return false;
}

// Function to process a received packet
void processPacket(const std::string& deviceType, uint8_t* packet) {
    // Verify checksum
    uint8_t checksum = 0;
    for (int i = 1; i < 5; i++) {
        checksum ^= packet[i];
    }

    if (checksum != packet[5]) {
        printf("[%s] Checksum error\n", deviceType.c_str());
        return;
    }

    // Extract float value (little-endian)
    float value;
    memcpy(&value, &packet[1], sizeof(float));

    printf("[%s] Received float value: %f\n", deviceType.c_str(), value);

    // Here you can process the data differently based on which device it came from
    if (deviceType == "TEENSY1") {
        // Process TEENSY1 data
    } else if (deviceType == "TEENSY2") {
        // Process TEENSY2 data
    }
}

// Thread function to handle data from a device
void* deviceThread(void* arg) {
    std::string deviceType = *(std::string*)arg;
    delete (std::string*)arg;

    // Thread loop
    while (g_running) {
        // Get device info safely
        DeviceInfo* device = nullptr;
        {
            std::lock_guard<std::mutex> lock(device_mutex);
            auto it = devices.find(deviceType);
            if (it == devices.end()) {
                break;  // Device no longer exists
            }
            device = it->second.get();
        }

        // Check if thread should exit
        {
            std::lock_guard<std::mutex> lock(device->mutex);
            if (!device->running) {
                break;
            }
        }

        // Check if device is connected
        bool is_connected;
        int fd;
        {
            std::lock_guard<std::mutex> lock(device->mutex);
            is_connected = device->connected;
            fd = device->fd;
        }

        if (!is_connected || fd == -1) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Check for device reset (sending identifier again)
        {
            std::string resetDeviceType;
            if (checkForIdentifier(fd, resetDeviceType)) {
                if (resetDeviceType == deviceType) {
                    printf("[%s] Device reset detected, sending ACK\n", deviceType.c_str());

                    // Send acknowledgment
                    const char* ack = "ACK\n";
                    write(fd, ack, strlen(ack));

                    // Update last activity
                    std::lock_guard<std::mutex> lock(device->mutex);
                    device->lastActivity = std::chrono::steady_clock::now();
                }

                // Skip this iteration to avoid processing the identifier as data
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }

        // Read data
        uint8_t buffer[BUFFER_SIZE];
        int bytesRead = read(fd, buffer, sizeof(buffer));

        if (bytesRead > 0) {
            // Update activity timestamp
            {
                std::lock_guard<std::mutex> lock(device->mutex);
                device->lastActivity = std::chrono::steady_clock::now();
            }

            static uint8_t packet[PACKET_SIZE];
            static int packetIndex = 0;
            static bool inPacket = false;

            // Process each byte
            for (int i = 0; i < bytesRead; i++) {
                uint8_t byte = buffer[i];

                if (!inPacket && byte == START_MARKER) {
                    inPacket = true;
                    packetIndex = 0;
                    packet[packetIndex++] = byte;
                }
                else if (inPacket) {
                    packet[packetIndex++] = byte;

                    if (byte == END_MARKER && packetIndex == PACKET_SIZE) {
                        processPacket(deviceType, packet);
                        inPacket = false;
                    }
                    else if (packetIndex >= PACKET_SIZE) {
                        inPacket = false;  // Reset if packet is too long
                    }
                }
            }
        }
        else if (bytesRead < 0) {
            // Error reading from device
            perror("Error reading from device");

            std::lock_guard<std::mutex> lock(device->mutex);
            device->connected = false;
            close(device->fd);
            device->fd = -1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[%s] Thread stopped\n", deviceType.c_str());
    return NULL;
}

// Function to scan for and connect to devices
void scanForDevices() {
    std::vector<std::string> ttyDevices = findTtyUsbDevices();
    std::vector<std::string> foundDevices;

    printf("Found %zu potential TTY devices\n", ttyDevices.size());

    // First, check existing devices for resets
    {
        std::lock_guard<std::mutex> lock(device_mutex);
        for (auto& pair : devices) {
            DeviceInfo* device = pair.second.get();
            std::lock_guard<std::mutex> dev_lock(device->mutex);

            if (device->connected && device->fd != -1) {
                std::string deviceType;
                if (checkForIdentifier(device->fd, deviceType)) {
                    // Device is sending identifier again - it probably reset
                    if (deviceType == device->type) {
                        printf("Device %s at %s has reset, sending ACK\n", 
                               device->type.c_str(), device->path.c_str());

                        // Send acknowledgment
                        const char* ack = "ACK\n";
                        write(device->fd, ack, strlen(ack));

                        // Update last activity
                        device->lastActivity = std::chrono::steady_clock::now();
                    }
                }
            }
        }
    }

    // Try to identify each device
    for (const auto& ttyDevice : ttyDevices) {
        // Check if this device is already connected
        bool alreadyConnected = false;
        {
            std::lock_guard<std::mutex> lock(device_mutex);
            for (auto& pair : devices) {
                DeviceInfo* device = pair.second.get();
                std::lock_guard<std::mutex> dev_lock(device->mutex);

                if (device->connected && device->path == ttyDevice) {
                    alreadyConnected = true;
                    foundDevices.push_back(device->type);
                    break;
                }
            }
        }

        if (alreadyConnected) {
            continue;
        }

        printf("Checking %s...\n", ttyDevice.c_str());

        int fd = openSerialPort(ttyDevice.c_str());
        if (fd != -1) {
            // Wait a bit for data to arrive
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Check for auto-identification
            std::string deviceType;
            if (checkForIdentifier(fd, deviceType)) {
                printf("Identified %s as %s (auto-ID)\n", ttyDevice.c_str(), deviceType.c_str());

                // Send acknowledgment
                const char* ack = "ACK\n";
                write(fd, ack, strlen(ack));
                printf("Sent ACK to %s\n", deviceType.c_str());

                // Lock before modifying the devices map
                std::lock_guard<std::mutex> lock(device_mutex);

                // Check if we already have this device type
                auto it = devices.find(deviceType);
                if (it != devices.end()) {
                    // We already have this device type
                    DeviceInfo* device = it->second.get();

                    std::lock_guard<std::mutex> dev_lock(device->mutex);

                    // If it's already connected, skip
                    if (device->connected) {
                        printf("%s is already connected at %s\n", 
                               deviceType.c_str(), device->path.c_str());
                        close(fd);
                    }
                    else {
                        // Update the device info
                        device->path = ttyDevice;
                        device->fd = fd;
                        device->connected = true;
                        device->lastActivity = std::chrono::steady_clock::now();
                        printf("%s reconnected at %s\n", deviceType.c_str(), ttyDevice.c_str());
                    }
                }
                else {
                    // New device type
                    auto device = std::make_unique<DeviceInfo>();
                    device->type = deviceType;
                    device->path = ttyDevice;
                    device->fd = fd;
                    device->connected = true;
                    device->running = true;
                    device->lastActivity = std::chrono::steady_clock::now();

                    // Start a thread for this device
                    std::string* threadArg = new std::string(deviceType);
                    pthread_create(&device->thread, NULL, deviceThread, threadArg);

                    // Store the device
                    devices[deviceType] = std::move(device);

                    printf("Started thread for %s\n", deviceType.c_str());
                }

                foundDevices.push_back(deviceType);
            }
            else {
                // Not one of our devices or not sending auto-ID
                close(fd);
            }
        }
    }

    // Check for disconnected devices
    std::lock_guard<std::mutex> lock(device_mutex);
    for (auto& pair : devices) {
        const std::string& deviceType = pair.first;
        DeviceInfo* device = pair.second.get();

        std::lock_guard<std::mutex> dev_lock(device->mutex);

        if (device->connected && 
            std::find(foundDevices.begin(), foundDevices.end(), deviceType) == foundDevices.end()) {
            printf("%s disconnected\n", deviceType.c_str());
            device->connected = false;
            close(device->fd);
            device->fd = -1;
        }
    }
}

// Function to get a device file descriptor by type (for external use)
int getDeviceFd(const std::string& deviceType) {
    std::lock_guard<std::mutex> lock(device_mutex);

    auto it = devices.find(deviceType);
    if (it != devices.end()) {
        std::lock_guard<std::mutex> dev_lock(it->second->mutex);
        if (it->second->connected) {
            return it->second->fd;
        }
    }

    return -1;
}

// Function to get a device path by type (for external use)
std::string getDevicePath(const std::string& deviceType) {
    std::lock_guard<std::mutex> lock(device_mutex);

    auto it = devices.find(deviceType);
    if (it != devices.end()) {
        std::lock_guard<std::mutex> dev_lock(it->second->mutex);
        if (it->second->connected) {
            return it->second->path;
        }
    }

    return "";
}

// Signal handler for clean shutdown
void signalHandler(int signum) {
    printf("Caught signal %d, cleaning up...\n", signum);

    g_running = false;

    // Stop all device threads
    {
        std::lock_guard<std::mutex> lock(device_mutex);
        for (auto& pair : devices) {
            DeviceInfo* device = pair.second.get();

            std::lock_guard<std::mutex> dev_lock(device->mutex);
            device->running = false;

            if (device->fd != -1) {
                close(device->fd);
                device->fd = -1;
            }
        }
    }

    // Wait a moment for threads to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    exit(signum);
}

// Background thread for device scanning
void* scanThread(void* arg) {
    while (g_running) {
        scanForDevices();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return NULL;
}

// Main function
int main() {
    printf("Integrated device receiver starting...\n");

    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Start background scanner thread
    pthread_t scanner_thread;
    pthread_create(&scanner_thread, NULL, scanThread, NULL);

    // Main loop - just wait for signals
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Print device status
        std::lock_guard<std::mutex> lock(device_mutex);
        printf("Connected devices:\n");
        for (auto& pair : devices) {
            DeviceInfo* device = pair.second.get();
            std::lock_guard<std::mutex> dev_lock(device->mutex);

            if (device->connected) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    now - device->lastActivity).count();

                printf("  %s: %s (fd=%d, last activity: %ld seconds ago)\n", 
                       device->type.c_str(), device->path.c_str(), device->fd, elapsed);
            }
        }
    }

    // Clean up
    pthread_join(scanner_thread, NULL);

    return 0;
}
