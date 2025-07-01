// In SerialParserWorker.h
#ifndef SERIALPARSERWORKER_H
#define SERIALPARSERWORKER_H

#include <QObject>
#include <QSerialPort>
#include <QByteArray>
#include <QReadWriteLock>

class SerialParserWorker : public QObject
{
    Q_OBJECT
public:
    explicit SerialParserWorker(QObject *parent = nullptr);
    ~SerialParserWorker();

    void startReading(const QString &portName);
    void stop();

    float getLatestValue();

signals:
    void packetReceived(float value);
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorString);  // Keep this signal here

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

private:
    void checkAndProcessData();
    void processPacket();

    QSerialPort serialPort;
    QByteArray holdingBuffer;
    QByteArray messageBuffer;
    int holdingBufferIndex;
    float latestValue;
    enum class ReceiveDataPacketStatus { IDLE, RECEIVING_DATA } receiveDataPacketState;
    QReadWriteLock valueLock;

    const char startMarker;
    const char endMarker;
};

#endif // SERIALPARSERWORKER_H
