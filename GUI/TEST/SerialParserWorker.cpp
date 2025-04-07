#include "SerialProtocol.h"
#include "SerialParserWorker.h"
#include <QDebug>
#include <QtCore/qmath.h>
#include <cstring>

SerialParserWorker::SerialParserWorker(QObject *parent)
    : QObject(parent),
      holdingBufferIndex(0),
      latestValue(0.0f),
      receiveDataPacketState(ReceiveDataPacketStatus::IDLE),
      startMarker(START_MARKER),
      endMarker(END_MARKER)
{
    holdingBuffer.reserve(HOLDING_BUFFER_SIZE);
    messageBuffer.reserve(MESSAGE_SIZE);
    
    // Connect error signal once during construction
    connect(&serialPort, static_cast<void(QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
            this, &SerialParserWorker::handleError);
}

SerialParserWorker::~SerialParserWorker() {
    stop();
}

void SerialParserWorker::startReading(const QString &portName)
{
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

    // Set serial parameters using values from SerialProtocol.h
    serialPort.setBaudRate(SERIAL_BAUD_RATE);
    serialPort.setDataBits(QSerialPort::DataBits(SERIAL_DATA_BITS));
    serialPort.setStopBits(QSerialPort::StopBits(SERIAL_STOP_BITS));
    serialPort.setParity(QSerialPort::Parity(SERIAL_PARITY_NONE));
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort.open(QIODevice::ReadWrite)) {
        emit errorOccurred(serialPort.error(), serialPort.errorString());
        return;
    }

    qDebug() << "[SerialParserWorker] Serial port opened on" << portName;

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

    qDebug() << "handleReadyRead called";

    QByteArray data = serialPort.readAll();

    qDebug() << "Received Data" << data.toHex();

    for (char byte : data) {
        if (holdingBuffer.size() < 1024) {
            holdingBuffer.append(byte);
            qDebug() << "append called";
        } else {
            holdingBuffer[holdingBufferIndex] = byte;
            holdingBufferIndex = (holdingBufferIndex + 1) % 1024;
            qDebug() << "overflow";
        }
        checkAndProcessData();
    }
}

void SerialParserWorker::checkAndProcessData() {

     qDebug() << "checkAndProcessData called";

    while (!holdingBuffer.isEmpty()) {
        uint8_t byte = static_cast<uint8_t>(holdingBuffer[0]);

        if (receiveDataPacketState == ReceiveDataPacketStatus::IDLE) {
            qDebug() << "-> IDLE";
            if (byte == startMarker) {
                messageBuffer.clear();
                messageBuffer.append(byte);
                receiveDataPacketState = ReceiveDataPacketStatus::RECEIVING_DATA;
                qDebug() << "!Start marker detected.";
            }
            qDebug() << "discard" << byte;
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