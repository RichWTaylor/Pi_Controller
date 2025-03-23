#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <vector>

#define SERIAL_PORT "/dev/serial0"  // Update if using a different port

using Buffer = std::vector<uint8_t>;

enum class ReceiveDataPacketStatus {
    IDLE,
    RECEIVING_DATA,
    TIME_OUT,
    ERROR        
} receiveDataPacketState;

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
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

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

#define BUFFER_SIZE 6
Buffer buffer(BUFFER_SIZE, 0);
int fd;
size_t idx = 0;

#define START_MARKER '<'
#define END_MARKER '>'

void addCharToBuffer(int data) {
    if (idx >= BUFFER_SIZE) return;  // Prevent buffer overflow
    buffer[idx] = data;
    idx++;
}

void resetBuffer() {
    idx = 0;
    std::fill(buffer.begin(), buffer.end(), 0);
}

void processPacket() {
    printf("-> processPacket :: Value byte 0: '%u'\n", buffer[0]);
    printf("-> processPacket :: Value byte 1: '%u'\n", buffer[1]);
    printf("-> processPacket :: Value byte 2: '%u'\n", buffer[2]);
    printf("-> processPacket :: Value byte 3: '%u'\n", buffer[3]);
    printf("-> processPacket :: Value byte 4: '%u'\n", buffer[4]);
    printf("\n");
}

void checkForDataPackets() {
    int bytesRead = read(fd, buffer.data(), 1);  // Read into the buffer
    if (bytesRead <= 0) return;  // If no data read, return

   // printf("Received bytes: '%u'\n", buffer.size());  // Print each byte in decimal

    for (int i = 0; i < bytesRead; i++) {
        uint8_t data = buffer[0];  // Current byte from the buffer

        switch (receiveDataPacketState) {
            case ReceiveDataPacketStatus::IDLE:
                if (data != START_MARKER) return;
                addCharToBuffer(data);
                receiveDataPacketState = ReceiveDataPacketStatus::RECEIVING_DATA;
                break;

            case ReceiveDataPacketStatus::RECEIVING_DATA:
                if ((data == '\n' || data == '\r') && idx == 0) return;  // Ignore newline/carriage return only outside a message
                addCharToBuffer(data);

                if (data == END_MARKER) {
                    processPacket();
                    resetBuffer();
                    receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
                }
                return;

            case ReceiveDataPacketStatus::TIME_OUT:
                break;

            case ReceiveDataPacketStatus::ERROR:
                break;
        }
    }
}

int main() {
    fd = openSerialPort(SERIAL_PORT);
    if (fd == -1) {
        return 1;
    }

    printf("Listening on %s...\n", SERIAL_PORT);

    while (1) {
        checkForDataPackets();
        usleep(100);  // Sleep for .1ms to avoid excessive CPU usage
    }

    close(fd);
    return 0;
}
