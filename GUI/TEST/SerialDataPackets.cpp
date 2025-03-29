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
    buffer.append(data);
    parseBuffer();
}

void SerialDataPackets::parseBuffer() {
    while (true) {
        int startIdx = buffer.indexOf(startMarker);
        int endIdx = buffer.indexOf(endMarker, startIdx + 1);

        if (startIdx == -1 || endIdx == -1) break;

        QByteArray packet = buffer.mid(startIdx, endIdx - startIdx + 1);
        if (packet.size() == 7) {
            float value;
            uint8_t reorder[4] = {static_cast<uint8_t>(packet[4]),
                                  static_cast<uint8_t>(packet[3]),
                                  static_cast<uint8_t>(packet[2]),
                                  static_cast<uint8_t>(packet[1])};
            memcpy(&value, reorder, sizeof(float));
            emit packetReceived(value);
        }

        buffer.remove(0, endIdx + 1);
    }
}
