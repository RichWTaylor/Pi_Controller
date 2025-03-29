#include "SerialDataPackets.h"
#include <QDebug>
#include <cstring>

SerialDataPackets::SerialDataPackets(QObject *parent) : QObject(parent) {
    connect(&serialHandler, &SerialHandler::errorOccurred, this, &SerialDataPackets::errorOccurred);

    serialHandler.setDataCallback([this](const QByteArray &data) {
        handleIncomingData(data);
    });
}

void SerialDataPackets::start(const QString &portName) {
    if (!serialHandler.open(portName)) {
        emit errorOccurred("Failed to open serial port");
    }
}

void SerialDataPackets::setMarkers(char start, char end) {
    startMarker = start;
    endMarker = end;
}

void SerialDataPackets::pushToCircularBuffer(uint8_t data) {
    int next = (head + 1) % BUFFER_SIZE;
    if (next == tail) {
        // Buffer full, overwrite oldest data
        tail = (tail + 1) % BUFFER_SIZE;
    }
    circularBuffer[head] = data;
    head = next;
}

bool SerialDataPackets::popFromCircularBuffer(uint8_t &data) {
    if (isCircularBufferEmpty()) return false;
    data = circularBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    return true;
}

bool SerialDataPackets::isCircularBufferEmpty() const {
    return head == tail;
}

int SerialDataPackets::circularBufferCount() const {
    return (head - tail + BUFFER_SIZE) % BUFFER_SIZE;
}

void SerialDataPackets::handleIncomingData(const QByteArray &data) {
    for (uint8_t byte : data) {
        pushToCircularBuffer(byte);
    }
    parseBuffer();
}

void SerialDataPackets::parseBuffer() {
    while (true) {
        int count = circularBufferCount();
        if (count < 7) return;  // Not enough data

        // Find start marker
        int startIdx = -1;
        for (int i = 0; i < count; ++i) {
            int index = (tail + i) % BUFFER_SIZE;
            if (circularBuffer[index] == static_cast<uint8_t>(startMarker)) {
                startIdx = index;
                break;
            }
        }
        if (startIdx == -1) return;

        // Find end marker
        int endIdx = -1;
        for (int i = 1; i < count; ++i) {
            int index = (startIdx + i) % BUFFER_SIZE;
            if (circularBuffer[index] == static_cast<uint8_t>(endMarker)) {
                endIdx = index;
                break;
            }
        }
        if (endIdx == -1) return;

        // Calculate packet size
        int packetSize = (endIdx - startIdx + BUFFER_SIZE) % BUFFER_SIZE + 1;
        if (packetSize != 7) {
            // Skip invalid packet
            tail = (endIdx + 1) % BUFFER_SIZE;
            continue;
        }

        // Extract packet
        QByteArray packet;
        for (int i = 0; i < packetSize; ++i) {
            int index = (startIdx + i) % BUFFER_SIZE;
            packet.append(circularBuffer[index]);
        }

        // Decode float
        float value;
        uint8_t reorder[4] = {static_cast<uint8_t>(packet[4]),
                              static_cast<uint8_t>(packet[3]),
                              static_cast<uint8_t>(packet[2]),
                              static_cast<uint8_t>(packet[1])};
        memcpy(&value, reorder, sizeof(float));
        emit packetReceived(value);

        // Move tail past processed packet
        tail = (endIdx + 1) % BUFFER_SIZE;
    }
}
