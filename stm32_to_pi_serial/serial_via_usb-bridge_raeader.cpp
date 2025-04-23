#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <vector>
#include <cstring>

#define SERIAL_PORT "/dev/ttyUSB0"  // find via  ls -l /dev/tty* & dmesg | grep tty

// sudo ln -s /dev/ttyAMA0 /dev/serial0

using Buffer = std::vector<uint8_t>;

enum class ReceiveDataPacketStatus {
    IDLE,
    RECEIVING_DATA,
    TIME_OUT,
    ERROR        
};

// Initialize the state to IDLE
ReceiveDataPacketStatus receiveDataPacketState = ReceiveDataPacketStatus::IDLE;

void clearBuffer(int fd) {
    tcflush(fd, TCIFLUSH);  // Flush the input buffer
}

// Function to open and configure the serial port
int openSerialPort(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY);
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

    clearBuffer(fd);

    // Configure baud rate
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Configure 8N1 (8 data bits, No parity, 1 stop bit)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem control lines
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
    options.c_oflag &= ~OPOST;  // Raw output

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("Error applying serial settings");
        close(fd);
        return -1;
    }

    return fd;
}

// Buffer settings
#define BUFFER_SIZE 7
Buffer buffer(BUFFER_SIZE, 0);
int fd;
size_t idx = 0;

#define START_MARKER 0x3C
#define END_MARKER 0x3E

// Function to safely add a character to the buffer
void addCharToBuffer(uint8_t data) {
    if (idx >= BUFFER_SIZE) return;  // Prevent buffer overflow
    buffer[idx] = data;
    idx++;
}

// Function to reset the buffer
void resetBuffer() {
    idx = 0;
    std::fill(buffer.begin(), buffer.end(), 0);
}

// Function to process received packet
void processPacket() {
    // Extract bytes in the same order they were sent (little-endian)
    uint8_t float_bytes[4] = {buffer[1], buffer[2], buffer[3], buffer[4]};

    // Convert directly to float (no reordering needed since both are little-endian)
    float fVal;
    memcpy(&fVal, float_bytes, sizeof(float));

    printf("Received float value: %f\n", fVal);
}



// Function to read and process incoming data packets
void checkForDataPackets() {
    uint8_t data;
    int bytesRead = read(fd, &data, 1);  // Read a single byte
    if (bytesRead <= 0) return;  // If no data read, return

    //printf("Received byte: '%u' (ASCII: '%c')\n", data, data);  // Debug output

    switch (receiveDataPacketState) {
        case ReceiveDataPacketStatus::IDLE:
            if (data != START_MARKER) return;
            resetBuffer();  // Reset buffer at start of new packet
            addCharToBuffer(data);
            receiveDataPacketState = ReceiveDataPacketStatus::RECEIVING_DATA;
            break;

        case ReceiveDataPacketStatus::RECEIVING_DATA:
            if ((data == '\n' || data == '\r') && idx == 0) return;  // Ignore newlines outside a message
            addCharToBuffer(data);

            if (data == END_MARKER) {
                if (idx == BUFFER_SIZE) {
                    processPacket();
                } else {
                    printf("(!) Invalid packet size: %zu bytes\n", idx);
                }
                resetBuffer();
                receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
            }
            
            break;

        case ReceiveDataPacketStatus::TIME_OUT:
        case ReceiveDataPacketStatus::ERROR:
            break;
    }
}

// Main function
int main() {
    fd = openSerialPort(SERIAL_PORT);
    if (fd == -1) {
        return 1;
    }

    printf("Listening on: %s...\n", SERIAL_PORT);

    while (1) {
        checkForDataPackets();
        usleep(100);  // Sleep for 0.1ms to avoid excessive CPU usage
    }

    close(fd);
    return 0;
}
