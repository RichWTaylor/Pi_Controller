#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <cstring>

// Function to open the serial port
int openSerialPort(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }
    return fd;
}

// Function to configure the serial port
bool configureSerialPort(int fd) {
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("Error getting serial port settings");
        return false;
    }

    cfsetispeed(&options, B115200);  // Baud rate
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~PARENB;   // No parity
    options.c_cflag &= ~CSTOPB;   // 1 stop bit
    options.c_cflag &= ~CSIZE;    // Clear current data size setting
    options.c_cflag |= CS8;       // 8 data bits
    options.c_cflag &= ~CRTSCTS;  // No flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver and set local mode

    // Apply settings
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("Error setting serial port settings");
        return false;
    }
    return true;
}

// Function to calculate checksum
uint8_t calculateChecksum(uint16_t value) {
    uint8_t checksum = 0;
    checksum ^= (uint8_t)(value & 0xFF);  // Lower byte
    checksum ^= (uint8_t)((value >> 8) & 0xFF);  // Upper byte
    return checksum;
}

// Function to read a packet from the serial port
bool readSerialData(int fd) {
    uint8_t buffer[5];  // To hold one packet (start marker, data, checksum, end marker)
    
    int bytesRead = read(fd, buffer, sizeof(buffer));
    if (bytesRead < 0) {
        perror("Error reading from serial port");
        return false;
    }

    // Check if the packet has the correct start and end markers
    if (buffer[0] != '<' || buffer[4] != '>') {
        // Using printf instead of std::cerr
        printf("Invalid packet: Incorrect start or end marker\n");
        return false;
    }

    // Extract 16-bit value from the packet (lower byte, then upper byte)
    uint16_t value = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
    uint8_t checksumReceived = buffer[3];

    // Verify checksum
    uint8_t checksumCalculated = calculateChecksum(value);
    if (checksumReceived != checksumCalculated) {
        // Using printf instead of std::cerr
        printf("Checksum error\n");
        return false;
    }

    // Successfully parsed packet
    printf("Received Value: %u\n", value);  // Use printf instead of std::cout
    return true;
}

int main() {
    const char* serialPort = "/dev/serial0";  // Update with your serial port name
    int fd = openSerialPort(serialPort);
    if (fd == -1) {
        return -1;
    }

    if (!configureSerialPort(fd)) {
        close(fd);
        return -1;
    }

    while (true) {
        if (!readSerialData(fd)) {
            // Using printf instead of std::cerr
            printf("Failed to read valid data\n");
        }
        usleep(100000); // Sleep for 100ms (adjust as needed)
    }

    close(fd);
    return 0;
}
