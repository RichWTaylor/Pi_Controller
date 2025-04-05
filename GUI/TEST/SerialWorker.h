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
    void processByte(uint8_t byte);  // Processes each byte received
    void processPacket();  // Extracts and parses a full packet

    QSerialPort serialHandler;
    QByteArray buffer;
    mutable QReadWriteLock valueLock;
    QReadWriteLock valueLock;  // To protect latestValue

    const char startMarker = '<';
    const char endMarker = '>';
    static constexpr int BUFFER_SIZE = 7;  // Matches C implementation

    enum class ReceiveDataPacketStatus {
        IDLE,
        RECEIVING_DATA,
        TIME_OUT,
        ERROR
    };

    ReceiveDataPacketStatus receiveDataPacketState = ReceiveDataPacketStatus::IDLE;
    float latestValue = 0.0f;
};

#endif // SERIALWORKER_H
