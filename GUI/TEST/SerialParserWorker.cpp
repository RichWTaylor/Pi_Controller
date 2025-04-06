#include "SerialParserWorker.h"
#include <QDebug>
#include <QtCore/qmath.h>
#include <cstring>

SerialParserWorker::SerialParserWorker(QObject *parent)
    : QObject(parent),
      holdingBufferIndex(0),
      latestValue(0.0f),
      receiveDataPacketState(ReceiveDataPacketStatus::IDLE),
      startMarker('<'),
      endMarker('>')
{
    holdingBuffer.reserve(1024);
    messageBuffer.reserve(1024);
}

SerialParserWorker::~SerialParserWorker() {
    stop();
}

void SerialParserWorker::startReading(const QString &portName)
{
    connect(&serialPort, &QSerialPort::readyRead, this, &SerialParserWorker::handleReadyRead);
    
    if (portName.isEmpty()) {
        emit errorOccurred(QSerialPort::UnknownError, "Port name is empty.");
        return;
    }

    // Check if the port is already open
    if (serialPort.isOpen()) {
        serialPort.close();  // Close it if already open
        qDebug() << "Serial port closed as it was already open.";
    }

    // Now open the port
    serialPort.setPortName(portName);
    if (!serialPort.open(QIODevice::ReadWrite)) {
        emit errorOccurred(serialPort.error(), serialPort.errorString());
        return;
    }

    qDebug() << "Serial port opened on" << portName;

    // Connect the `readyRead` signal to `handleReadyRead` method
    connect(&serialPort, &QSerialPort::readyRead, this, &SerialParserWorker::handleReadyRead);
}


void SerialParserWorker::stop() {
    if (serialPort.isOpen()) {
        serialPort.close();
        qDebug() << "Serial port closed.";
    }
}

float SerialParserWorker::getLatestValue() {
    QReadLocker locker(&valueLock);
    return latestValue;
}

void SerialParserWorker::handleReadyRead() {
    QByteArray data = serialPort.readAll();
    for (char byte : data) {
        if (holdingBuffer.size() < 1024) {
            holdingBuffer.append(byte);
        } else {
            holdingBuffer[holdingBufferIndex] = byte;
            holdingBufferIndex = (holdingBufferIndex + 1) % 1024;
        }
        checkAndProcessData();
    }
}

void SerialParserWorker::checkAndProcessData() {
    while (!holdingBuffer.isEmpty()) {
        uint8_t byte = static_cast<uint8_t>(holdingBuffer[0]);

        if (receiveDataPacketState == ReceiveDataPacketStatus::IDLE) {
            if (byte == startMarker) {
                messageBuffer.clear();
                messageBuffer.append(byte);
                receiveDataPacketState = ReceiveDataPacketStatus::RECEIVING_DATA;
                qDebug() << "Start marker detected.";
            }
            holdingBuffer.remove(0, 1);
        } else if (receiveDataPacketState == ReceiveDataPacketStatus::RECEIVING_DATA) {
            messageBuffer.append(byte);
            if (messageBuffer.size() == 7) {
                if (!messageBuffer.isEmpty() && messageBuffer.back() == endMarker) {
                    processPacket();
                } else {
                    qWarning() << "Invalid packet (no end marker).";
                }
                messageBuffer.clear();
                receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
            }
            holdingBuffer.remove(0, 1);
        }
    }
}

void SerialParserWorker::processPacket() {
    if (messageBuffer.size() != 7) return;

    float fVal;
    uint8_t reorder[4] = {
        static_cast<uint8_t>(messageBuffer[4]),
        static_cast<uint8_t>(messageBuffer[3]),
        static_cast<uint8_t>(messageBuffer[2]),
        static_cast<uint8_t>(messageBuffer[1])
    };

    std::memcpy(&fVal, reorder, sizeof(fVal));

    if (qIsNaN(fVal)) {
        qWarning() << "Received NaN value.";
        return;
    }

    {
        QWriteLocker locker(&valueLock);
        latestValue = fVal;
    }

    qDebug() << "Decoded float value:" << fVal;
    emit packetReceived(fVal);
}

void SerialParserWorker::handleError(QSerialPort::SerialPortError error) {
    if (error != QSerialPort::NoError) {
        emit errorOccurred(error, serialPort.errorString());
    }
}
