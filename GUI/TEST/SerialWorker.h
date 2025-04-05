#ifndef SERIALWORKER_H
#define SERIALWORKER_H

#include <QObject>
#include <QSerialPort>
#include <QByteArray>
#include <QReadWriteLock>
#include <QDebug>
#include <QThread>
#include <cstring>

class SerialWorker : public QObject
{
    Q_OBJECT

public:
    explicit SerialWorker(QObject *parent = nullptr);
    ~SerialWorker();

    void startReading(const QString &portName);  // Starts the serial communication
    void stopReading();                           // Stops the serial communication
    float getLatestValue() const;  // Add this method to access latestValue safely

signals:
    void packetReceived(float value);  // Emitted when a packet is parsed
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorMessage);

private slots:
    void handleIncomingData();  // Handles incoming data byte-by-byte
    void handleError(QSerialPort::SerialPortError error); // Handles errors

private:
    void checkAndProcessData();  // Processes data from the holding buffer and moves to message buffer
    void processByte(uint8_t byte);  // Processes each byte received
    void processPacket();  // Extracts and parses a full packet

    QSerialPort serialHandler;
    QByteArray holdingBuffer;
    QByteArray messageBuffer;
    mutable QReadWriteLock valueLock;

    const char startMarker = '<';
    const char endMarker = '>';
    static constexpr int HOLDING_BUFFER_SIZE = 1024;  // Size for the holding buffer
    static constexpr int MESSAGE_BUFFER_SIZE = 8;  // Fixed size for message buffer

    enum class ReceiveDataPacketStatus {
        IDLE,
        RECEIVING_DATA,
        TIME_OUT,
        ERROR
    };

    ReceiveDataPacketStatus receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
    float latestValue = 0.0f;

    int holdingBufferIndex = 0; // Used for overwriting the oldest data in the holding buffer
};

#endif // SERIALWORKER_H
