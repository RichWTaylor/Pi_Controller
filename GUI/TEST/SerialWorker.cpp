#include "SerialWorker.h"
#include <QDebug>
#include <cstring>

#define HOLDING_BUFFER_SIZE 1000
#define MESSAGE_BUFFER_SIZE 7

SerialWorker::SerialWorker(QObject *parent)
    : QObject(parent),
      startMarker('<'),
      endMarker('>'),
      receiveDataPacketState(ReceiveDataPacketStatus::IDLE),
      latestValue(0.0f)
{
    qDebug() << "SerialWorker constructor called.";

    connect(&serialHandler, &QSerialPort::readyRead, this, &SerialWorker::handleIncomingData);
    connect(&serialHandler, &QSerialPort::errorOccurred, this, &SerialWorker::handleError);

    holdingBuffer.reserve(HOLDING_BUFFER_SIZE);
    messageBuffer.reserve(MESSAGE_BUFFER_SIZE);
}

SerialWorker::~SerialWorker()
{
    qDebug() << "SerialWorker destructor called!";
    stopReading();
}

void SerialWorker::startReading(const QString &portName)
{
    qDebug() << "Opening serial port:" << portName;
    serialHandler.setPortName(portName);
    serialHandler.setBaudRate(QSerialPort::Baud115200);
    serialHandler.setDataBits(QSerialPort::Data8);
    serialHandler.setParity(QSerialPort::NoParity);
    serialHandler.setStopBits(QSerialPort::OneStop);
    serialHandler.setFlowControl(QSerialPort::NoFlowControl);

    if (!serialHandler.open(QIODevice::ReadWrite)) {
        qCritical() << "Failed to open serial port:" << serialHandler.errorString();
        emit errorOccurred(QSerialPort::UnknownError, "Failed to open the serial port.");
    } else {
        qDebug() << "Serial port opened successfully.";
        serialHandler.clear();
        holdingBuffer.clear();
        receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
        qDebug() << "Holding buffer flushed.";
    }
}

void SerialWorker::stopReading()
{
    qDebug() << "Closing serial port...";
    if (serialHandler.isOpen()) {
        serialHandler.close();
        qDebug() << "Serial port closed.";
    }
}

void SerialWorker::handleIncomingData()
{
    QByteArray data = serialHandler.readAll();

    if (data.isEmpty()) {
        qDebug() << "No data received.";
    } else {
        //qDebug() << "Data received:" << data.toHex();

        // Insert the received data into the holding buffer
        for (char byte : data) {
            if (holdingBuffer.size() < HOLDING_BUFFER_SIZE) {
                //qDebug() << "append";
                holdingBuffer.append(byte);
            } else {
                holdingBuffer[holdingBufferIndex] = byte;  // Overwrite the oldest data
                qDebug() << "Holding buffer full, overwriting oldest byte.";
                holdingBufferIndex = (holdingBufferIndex + 1) % HOLDING_BUFFER_SIZE;
            }
            // Check the holding buffer and move data to the message buffer
            checkAndProcessData();
        }

    }




}

void SerialWorker::checkAndProcessData()
{
    //qDebug() << "checkAndProcessData()";

    while (!holdingBuffer.isEmpty()) {
        uint8_t byte = static_cast<uint8_t>(holdingBuffer[0]);

        if (receiveDataPacketState == ReceiveDataPacketStatus::IDLE) {
            if (byte == startMarker) {
                messageBuffer.clear();
                messageBuffer.append(byte);
                receiveDataPacketState = ReceiveDataPacketStatus::RECEIVING_DATA;
                qDebug() << "Start marker observed, transitioning to RECEIVING_DATA state.";
            } else {
                holdingBuffer.remove(0, 1);  // Discard byte if not a start marker in IDLE state
                qDebug() << "byte: ";
                qDebug() << byte;
                qDebug() << "discarded";
            }
        } else if (receiveDataPacketState == ReceiveDataPacketStatus::RECEIVING_DATA) {
            messageBuffer.append(byte);
            qDebug() << "append byte to messageBuffer: ";
            qDebug() << byte;
            holdingBuffer.remove(0, 1);  // Remove processed byte

            if (messageBuffer.size() == MESSAGE_BUFFER_SIZE) {
                // Use 'at()' to access the last element of the QByteArray
                qDebug() << "check end marker:";
                if (!messageBuffer.isEmpty() && messageBuffer.at(messageBuffer.size()-1) == endMarker) {
                    processPacket();  // Valid packet
                } else {
                    qWarning() << "Invalid packet received, discarding message.";
                    messageBuffer.clear();
                }

                // Reset the state to IDLE after processing
                receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
            }
        }
    }
}


#include <QtCore/qmath.h>  // Make sure to include this for qIsNaN()

void SerialWorker::processPacket()
{
    if (messageBuffer.size() != MESSAGE_BUFFER_SIZE) {
        qWarning() << "Invalid packet size:" << messageBuffer.size();
        return;
    }

    // Check if the last byte is the end marker '>'
    if (messageBuffer.at(messageBuffer.size() - 1) == endMarker) {
        float fVal;
        uint8_t reorder[4] = {
            static_cast<uint8_t>(messageBuffer[4]),
            static_cast<uint8_t>(messageBuffer[3]),
            static_cast<uint8_t>(messageBuffer[2]),
            static_cast<uint8_t>(messageBuffer[1])
        };

        std::memcpy(&fVal, reorder, sizeof(fVal));

        if (qIsNaN(fVal)) {  // Use qIsNaN() instead of std::isnan()
            qWarning() << "Received NaN value — ignoring.";
            return;
        }

        {
            QWriteLocker locker(&valueLock);
            latestValue = fVal;
        }

        qDebug() << "Decoded float value:" << fVal;
        emit packetReceived(fVal);
    } else {
        qWarning() << "Invalid packet received, discarding message.";
        messageBuffer.clear();
    }
}


void SerialWorker::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError) return;

    QString errorMessage;
    switch (error) {
        case QSerialPort::DeviceNotFoundError: errorMessage = "Device not found."; break;
        case QSerialPort::PermissionError: errorMessage = "Permission error."; break;
        case QSerialPort::OpenError: errorMessage = "Failed to open port."; break;
        case QSerialPort::ReadError: errorMessage = "Read error."; break;
        case QSerialPort::WriteError: errorMessage = "Write error."; break;
        default: errorMessage = "Unknown error."; break;
    }

    qCritical() << "Serial error:" << errorMessage;
    emit errorOccurred(error, errorMessage);
}
