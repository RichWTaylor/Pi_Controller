#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#define SERIAL_PORT "/dev/serial0"  // Update if using a different port

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

int main() {
    int fd = openSerialPort(SERIAL_PORT);
    if (fd == -1) {
        return 1;
    }

    printf("Listening on %s...\n", SERIAL_PORT);

    uint8_t buffer[256];  // Buffer for incoming data
    while (1) {
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';  // Null-terminate for printing
            printf("Received: %s\n", buffer);
        } else if (bytesRead < 0) {
            perror("Error reading serial data");
            break;
        }
        usleep(100000);  // Sleep for 100ms to avoid excessive CPU usage
    }

    close(fd);
    return 0;
}
