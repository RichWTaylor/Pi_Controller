// SerialWorker.cpp
#include "SerialWorker.h"


SerialWorker::SerialWorker(QObject *parent)
    : QObject(parent),
      startMarker('<'),
      endMarker('>'),
      latestValue(0.0)
{
    connect(&serialHandler, &QSerialPort::readyRead, this, &SerialWorker::handleIncomingData);
    connect(&serialHandler, &QSerialPort::errorOccurred, this, &SerialWorker::handleError);
}

SerialWorker::~SerialWorker()
{
    stopReading();
}

void SerialWorker::startReading(const QString &portName)
{
    serialHandler.setPortName(portName);
    serialHandler.setBaudRate(QSerialPort::Baud115200);
    serialHandler.setDataBits(QSerialPort::Data8);
    serialHandler.setParity(QSerialPort::NoParity);
    serialHandler.setStopBits(QSerialPort::OneStop);
    serialHandler.setFlowControl(QSerialPort::NoFlowControl);

    if (!serialHandler.open(QIODevice::ReadWrite)) {
        emit errorOccurred(QSerialPort::UnknownError, "Failed to open the serial port.");
    }
}

void SerialWorker::stopReading()
{
    if (serialHandler.isOpen()) {
        serialHandler.close();
    }
}

void SerialWorker::handleIncomingData()
{
    QByteArray data = serialHandler.readAll();
    for (char byte : data) {
        circularBuffer.append(byte);
        if (circularBuffer.size() > BUFFER_SIZE) {
            circularBuffer.remove(0, 1);
        }
    }
    parseBuffer();
}

void SerialWorker::parseBuffer()
{
    while (true) {
        int startIdx = circularBuffer.indexOf(startMarker);
        int endIdx = circularBuffer.indexOf(endMarker, startIdx + 1);

        if (startIdx == -1 || endIdx == -1) break;

        QByteArray packet = circularBuffer.mid(startIdx, endIdx - startIdx + 1);

        if (packet.size() == 7) {
            float value;
            uint8_t reorder[4] = {
                static_cast<uint8_t>(packet[4]),
                static_cast<uint8_t>(packet[3]),
                static_cast<uint8_t>(packet[2]),
                static_cast<uint8_t>(packet[1])
            };
            memcpy(&value, reorder, sizeof(float));

            {
                QWriteLocker locker(&valueLock);
                latestValue = value;
            }

            emit packetReceived(value);
        }

        // Remove processed packet
        circularBuffer.remove(0, endIdx + 1);
    }
}

void SerialWorker::handleError(QSerialPort::SerialPortError error)
{
    QString errorMessage;
    switch (error) {
    case QSerialPort::NoError:
        return;
    case QSerialPort::DeviceNotFoundError:
        errorMessage = "Device not found.";
        break;
    case QSerialPort::PermissionError:
        errorMessage = "Permission error.";
        break;
    case QSerialPort::OpenError:
        errorMessage = "Failed to open port.";
        break;
    case QSerialPort::ReadError:
        errorMessage = "Read error.";
        break;
    case QSerialPort::WriteError:
        errorMessage = "Write error.";
        break;
    default:
        errorMessage = "Unknown error.";
        break;
    }

    emit errorOccurred(error, errorMessage);
}
