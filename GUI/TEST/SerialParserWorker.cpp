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
    connect(&serial, &QSerialPort::readyRead, this, &SerialParserWorker::handleReadyRead);
    connect(&serial, &QSerialPort::errorOccurred, this, &SerialParserWorker::handleError);

    holdingBuffer.reserve(HOLDING_BUFFER_SIZE);
    messageBuffer.reserve(MESSAGE_BUFFER_SIZE);
}

SerialParserWorker::~SerialParserWorker() {
    stop();
}

void SerialParserWorker::start(const QString &portName, int baudRate) {
    serial.setPortName(portName);
    serial.setBaudRate(baudRate);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial.open(QIODevice::ReadWrite)) {
        emit errorOccurred(serial.error(), serial.errorString());
        return;
    }

    qDebug() << "Serial port opened successfully.";
    serial.clear();
    holdingBuffer.clear();
    receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
}

void SerialParserWorker::stop() {
    if (serial.isOpen()) {
        serial.close();
        qDebug() << "Serial port closed.";
    }
}

float SerialParserWorker::getLatestValue() {
    QReadLocker locker(&valueLock);
    return latestValue;
}

void SerialParserWorker::handleReadyRead() {
    QByteArray data = serial.readAll();
    for (char byte : data) {
        if (holdingBuffer.size() < HOLDING_BUFFER_SIZE) {
            holdingBuffer.append(byte);
        } else {
            holdingBuffer[holdingBufferIndex] = byte;
            holdingBufferIndex = (holdingBufferIndex + 1) % HOLDING_BUFFER_SIZE;
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
            if (messageBuffer.size() == MESSAGE_BUFFER_SIZE) {
                if (messageBuffer.last() == endMarker) {
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
    if (messageBuffer.size() != MESSAGE_BUFFER_SIZE) return;

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
        emit errorOccurred(error, serial.errorString());
    }
}
