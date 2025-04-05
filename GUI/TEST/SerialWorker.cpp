#include "SerialWorker.h"
#include <QDebug>
#include <cstring>

SerialWorker::SerialWorker(QObject *parent)
    : QObject(parent),
      startMarker('<'),
      endMarker('>'),
      receiveDataPacketState(ReceiveDataPacketStatus::IDLE),
      latestValue(0.0)
{
    qDebug() << "SerialWorker constructor called.";
   // serialHandler.moveToThread(this->thread());  // Move serialHandler to the worker thread
    connect(&serialHandler, &QSerialPort::readyRead, this, &SerialWorker::handleIncomingData);
    connect(&serialHandler, &QSerialPort::errorOccurred, this, &SerialWorker::handleError);
    buffer.reserve(100);
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

void SerialWorker::handleIncomingData() {
    QByteArray data = serialHandler.readAll();

    // Log the size of the incoming data
    qDebug() << "Received data size:" << data.size();
    if (data.isEmpty()) {
        qDebug() << "No data received.";
    } else {
        qDebug() << "Data received:" << data.toHex();
    }

    // Check for partial data and handle accordingly
    for (char byte : data) {
        // Log each byte being processed
        qDebug() << "Processing byte:" << static_cast<uint8_t>(byte);
        processByte(static_cast<uint8_t>(byte));
    }

    // Log the receive data packet state
    qDebug() << "ReceiveDataPacketStatus:" << static_cast<int>(receiveDataPacketState);

    // Handle buffer clear based on the state
    if (receiveDataPacketState == ReceiveDataPacketStatus::IDLE) {
        qDebug() << "Clearing buffer as state is IDLE";
        buffer.clear();  // Clear the buffer if we are in IDLE state
    }

    // Log the current buffer size and contents after processing
    qDebug() << "Current buffer size:" << buffer.size();
    qDebug() << "Current buffer contents:" << buffer.toHex();
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
                    buffer.clear();  // Clear buffer to reset the state
                    receiveDataPacketState = ReceiveDataPacketStatus::IDLE;  // Reset state
                }
            }
            break;
        

        case ReceiveDataPacketStatus::TIME_OUT:
        case ReceiveDataPacketStatus::ERROR:
            qWarning() << "Unexpected state, resetting...";
            buffer.clear();  // Clear the buffer to handle fragmented data correctly
            receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
            break;
    }
}

#include <cmath> // For std::isnan

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

    if (buffer.size() >= 5) {
        uint8_t reorder[4] = {
            static_cast<uint8_t>(buffer[4]),
            static_cast<uint8_t>(buffer[3]),
            static_cast<uint8_t>(buffer[2]),
            static_cast<uint8_t>(buffer[1])
        };
        std::memcpy(&fVal, reorder, sizeof(fVal));
    } else {
        qWarning() << "Buffer too small for float decode!";
        return;
    }


    if (std::isnan(fVal)) {
        qWarning() << "(!) Received NaN value — ignoring.";
        return;  // Skip NaNs entirely
    }

    {
        QWriteLocker locker(&valueLock);
        latestValue = fVal;
    }

    qDebug() << " |";
    qDebug() << "  -> Decoded float value:" << fVal;
    emit packetReceived(fVal);
}


float SerialWorker::getLatestValue() const {
    QReadLocker locker(&valueLock);  // Use read lock for thread-safe access
    return latestValue;
}
/*
Since you're not directly calling getLatestValue() in your main class and you're instead using the signal-slot mechanism to update the QML, 
this is already a solid design. However, if you do need to retrieve the latest value programmatically (outside of the signal-slot mechanism), 
calling getLatestValue() in SerialWorker from anywhere in the code (e.g., serialPackets->getLatestValue()) would be the way to go.
*/

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
    emit errorOccurred(error, errorMessage);  // Ensure correct connection type here
}

