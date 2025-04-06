#pragma once

#include <QObject>
#include <QSerialPort>
#include <QReadWriteLock>

class SerialParserWorker : public QObject {
    Q_OBJECT

public:
    explicit SerialParserWorker(QObject *parent = nullptr);
    ~SerialParserWorker();

    void start(const QString &portName, int baudRate = QSerialPort::Baud115200);
    void stop();

    float getLatestValue();

signals:
    void packetReceived(float value);
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorMessage);

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

private:
    void checkAndProcessData();
    void processPacket();

    QSerialPort serial;
    QByteArray holdingBuffer;
    QByteArray messageBuffer;
    int holdingBufferIndex;

    QReadWriteLock valueLock;
    float latestValue;

    enum class ReceiveDataPacketStatus { IDLE, RECEIVING_DATA };
    ReceiveDataPacketStatus receiveDataPacketState;

    const char startMarker;
    const char endMarker;

    static constexpr int HOLDING_BUFFER_SIZE = 1000;
    static constexpr int MESSAGE_BUFFER_SIZE = 7;
};
