#include "SerialDataPackets.h"
#include <QDebug>

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

void SerialDataPackets::handleIncomingData(const QByteArray &data) {
    for (int i = 0; i < data.size(); ++i) {
        pushToCircularBuffer(data[i]);
    }
    parseBuffer();
}

void SerialDataPackets::parseBuffer() {
    while (true) {
        int startIdx = circularBuffer.indexOf(startMarker);
        int endIdx = circularBuffer.indexOf(endMarker, startIdx + 1);

        if (startIdx == -1 || endIdx == -1) break;

        QByteArray packet = circularBuffer.mid(startIdx, endIdx - startIdx + 1);
        if (packet.size() == 7) {
            float value;
            uint8_t reorder[4] = {static_cast<uint8_t>(packet[4]),
                                  static_cast<uint8_t>(packet[3]),
                                  static_cast<uint8_t>(packet[2]),
                                  static_cast<uint8_t>(packet[1])};
            memcpy(&value, reorder, sizeof(float));
            emit packetReceived(value);
        }

        // Remove the processed packet from the circular buffer
        for (int i = 0; i <= endIdx; ++i) {
            readFromCircularBuffer();
        }
    }
}

void SerialDataPackets::pushToCircularBuffer(uint8_t data) {
    circularBuffer.append(data);

    // If the buffer exceeds the max size, remove the oldest data (circular behavior)
    if (circularBuffer.size() > BUFFER_SIZE) {
        circularBuffer.remove(0, 1);
    }
}

QByteArray SerialDataPackets::readFromCircularBuffer() {
    if (circularBuffer.isEmpty()) {
        return QByteArray();
    }

    QByteArray result = circularBuffer.left(1);
    circularBuffer.remove(0, 1);
    return result;
}
