// SerialWorker.h
#ifndef SERIALWORKER_H
#define SERIALWORKER_H

#include <QObject>
#include <QSerialPort>
#include <QByteArray>
#include <QReadWriteLock>
#include <QDebug>
#include <QThread>
class SerialWorker : public QObject
{
    Q_OBJECT

public:
    explicit SerialWorker(QObject *parent = nullptr);
    ~SerialWorker();

    void startReading(const QString &portName);  // Starts the serial communication
    void stopReading();                          // Stops the serial communication

signals:
    void packetReceived(float value);  // Emitted when a packet is parsed
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorMessage);

private slots:
    void handleIncomingData();  // Handles incoming data and parsing
    void handleError(QSerialPort::SerialPortError error); // Handles errors

private:
    void parseBuffer();  // Parses the buffer to extract packets

    QSerialPort serialHandler;
    QByteArray circularBuffer;
    char startMarker = '<';
    char endMarker = '>';
    static constexpr int BUFFER_SIZE = 1024;
    mutable QReadWriteLock valueLock;
    float latestValue = 0.0f;
};

#endif // SERIALWORKER_H
