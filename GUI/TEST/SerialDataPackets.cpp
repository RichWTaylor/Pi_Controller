#include "SerialDataPackets.h"
#include <QDebug>
#include <QThread>

SerialDataPackets::SerialDataPackets(QObject *parent) : QObject(parent), latestValue(0.0)
{
    connect(&serialHandler, &QSerialPort::readyRead, this, &SerialDataPackets::handleIncomingData);
    connect(&serialHandler, &QSerialPort::errorOccurred, this, &SerialDataPackets::errorOccurred);
}

SerialDataPackets::~SerialDataPackets() {
    cleanup();  // Ensure cleanup is done when the object is deleted
}

void SerialDataPackets::start(const QString &portName)
{
    serialHandler.setPortName(portName);
    if (!serialHandler.open(QIODevice::ReadWrite)) {
        emit errorOccurred("Failed to open serial port");
        return;
    }
    serialHandler.setBaudRate(QSerialPort::Baud115200);
    serialHandler.setDataBits(QSerialPort::Data8);
    serialHandler.setParity(QSerialPort::NoParity);
    serialHandler.setStopBits(QSerialPort::OneStop);
    serialHandler.setFlowControl(QSerialPort::NoFlowControl);
}

void SerialDataPackets::stop() {
    if (serialHandler.isOpen()) {
        serialHandler.close();
    }
}

void SerialDataPackets::setMarkers(char start, char end) {
    startMarker = start;
    endMarker = end;
}

void SerialDataPackets::handleIncomingData(const QByteArray &data)
{
    for (int i = 0; i < data.size(); ++i) {
        pushToCircularBuffer(data[i]);
    }
    parseBuffer();
}

void SerialDataPackets::parseBuffer()
{
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

            // Store the latest value thread-safely
            {
                QWriteLocker locker(&valueLock);
                latestValue = value;
            }

            emit packetReceived(value);
        }

        // Remove the processed packet from the circular buffer
        for (int i = 0; i <= endIdx; ++i) {
            readFromCircularBuffer();
        }
    }
}

void SerialDataPackets::pushToCircularBuffer(uint8_t data)
{
    circularBuffer.append(data);

    // If the buffer exceeds the max size, remove the oldest data (circular behavior)
    if (circularBuffer.size() > BUFFER_SIZE) {
        circularBuffer.remove(0, 1);
    }
}

QByteArray SerialDataPackets::readFromCircularBuffer()
{
    if (circularBuffer.isEmpty()) {
        return QByteArray();
    }

    QByteArray result = circularBuffer.left(1);
    circularBuffer.remove(0, 1);
    return result;
}

// Read the latest value thread-safely
float SerialDataPackets::getLatestValue() const
{
    QReadLocker locker(&valueLock);
    return latestValue;
}

void SerialDataPackets::cleanup()
{
    stop();  // Close the serial port if open
    // Any other cleanup operations can be done here
}
