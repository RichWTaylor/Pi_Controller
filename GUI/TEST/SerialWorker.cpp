#include "SerialWorker.h"
#include <QDebug>
#include <cstring>

SerialWorker::SerialWorker(QObject *parent)
    : QObject(parent),
      startMarker('<'),
      endMarker('>'),
      latestValue(0.0),
      receiveDataPacketState(ReceiveDataPacketStatus::IDLE)
{
    qDebug() << "SerialWorker constructor called.";
    connect(&serialHandler, &QSerialPort::readyRead, this, &SerialWorker::handleIncomingData);
    connect(&serialHandler, &QSerialPort::errorOccurred, this, &SerialWorker::handleError);
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

        // Flush any stale data
        serialHandler.clear();
        buffer.clear();
        receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
        qDebug() << "Serial buffer flushed.";
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
    //qDebug() << "Received raw data:" << data.toHex();

    for (char byte : data) {
        processByte(static_cast<uint8_t>(byte));
    }
}

// Packet parsing logic similar to your C code
void SerialWorker::processByte(uint8_t byte)
{
    switch (receiveDataPacketState) {
        case ReceiveDataPacketStatus::IDLE:
            if (byte == startMarker) {
                buffer.clear();
                buffer.append(byte);
                receiveDataPacketState = ReceiveDataPacketStatus::RECEIVING_DATA;
                qDebug() << " -> START MARKER OBSERVED";
            }
            break;

        case ReceiveDataPacketStatus::RECEIVING_DATA:
            buffer.append(byte);

            if (byte == endMarker) {
                qDebug() << " -> END MARKER OBSERVED";
                if (buffer.size() == BUFFER_SIZE) {
                    processPacket();
                } else {
                    qWarning() << "(!) Invalid packet size:" << buffer.size();
                    latestValue = -1; // safety value
                }
                receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
            }
            break;

        case ReceiveDataPacketStatus::TIME_OUT:
        case ReceiveDataPacketStatus::ERROR:
            qWarning() << "Unexpected state, resetting...";
            receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
            break;
    }
}

void SerialWorker::processPacket()
{
    if (buffer.size() != BUFFER_SIZE) {
        qWarning() << "Corrupted packet ignored. Size:" << buffer.size();
        return;
    }

    float fVal;
    uint8_t reorder[4] = {
        static_cast<uint8_t>(buffer[4]),
        static_cast<uint8_t>(buffer[3]),
        static_cast<uint8_t>(buffer[2]),
        static_cast<uint8_t>(buffer[1])
    };

    std::memcpy(&fVal, reorder, sizeof(fVal));

    {
        QWriteLocker locker(&valueLock);
        latestValue = fVal;
    }

    qDebug() << " |";
    qDebug() << "  -> Decoded float value:" << fVal;
    emit packetReceived(fVal);
}

void SerialWorker::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError)
        return;

    QString errorMessage;
    switch (error) {
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

    qCritical() << "Serial error:" << errorMessage;
    emit errorOccurred(error, errorMessage);
}
