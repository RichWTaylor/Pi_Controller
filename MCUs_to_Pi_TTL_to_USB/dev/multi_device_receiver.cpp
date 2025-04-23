g++// multi_device_receiver.cpp
// Compile with: g++ -o multi_device_receiver multi_device_receiver.cpp -pthread

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
#include <mutex>
#include <memory>
#include <signal.h>  // Added for signal handling

// Constants
#define BUFFER_SIZE 256
#define PACKET_SIZE 7
#define START_MARKER 0x3C
#define END_MARKER 0x3E
#define MAX_DEVICES 2

// Device information structure
struct DeviceInfo {
    std::string path;
    std::string type;
    int fd;
    pthread_t thread;
    bool running;
    bool connected;
    std::mutex mutex;  // For thread-safe access to non-atomic members

    DeviceInfo() : fd(-1), running(false), connected(false) {}

    // Non-copyable
    DeviceInfo(const DeviceInfo&) = delete;
    DeviceInfo& operator=(const DeviceInfo&) = delete;
};

// Global variables
std::map<std::string, std::unique_ptr<DeviceInfo>> devices;
std::mutex device_mutex;

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
    while (1) {
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
        {
            std::lock_guard<std::mutex> lock(device->mutex);
            is_connected = device->connected;
        }

        if (!is_connected) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Read data
        int fd;
        {
            std::lock_guard<std::mutex> lock(device->mutex);
            fd = device->fd;
        }

        uint8_t buffer[BUFFER_SIZE];
        int bytesRead = read(fd, buffer, sizeof(buffer));

        if (bytesRead > 0) {
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

    // Try to identify each device
    for (const auto& ttyDevice : ttyDevices) {
        printf("Checking %s...\n", ttyDevice.c_str());

        int fd = openSerialPort(ttyDevice.c_str());
        if (fd != -1) {
            std::string deviceType = identifyDevice(fd);

            if (!deviceType.empty()) {
                printf("Identified %s as %s\n", ttyDevice.c_str(), deviceType.c_str());

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
                // Not one of our devices
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

// Signal handler for clean shutdown
void signalHandler(int signum) {
    printf("Caught signal %d, cleaning up...\n", signum);

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

// Main function
int main() {
    printf("Multi-device TTL-USB receiver starting...\n");

    // Set up signal handling
    signal(SIGINT, signalHandler);   // Handle Ctrl+C
    signal(SIGTERM, signalHandler);  // Handle termination signal

    // Initial scan for devices
    scanForDevices();

    // Main loop - periodically scan for new/disconnected devices
    while (1) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        scanForDevices();
    }

    return 0;
}
