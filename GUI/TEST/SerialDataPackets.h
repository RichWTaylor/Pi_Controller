#ifndef SERIALDATAPACKETS_H
#define SERIALDATAPACKETS_H

#include <QObject>
#include <QByteArray>
#include <QSerialPort>
#include <QReadWriteLock>

class SerialDataPackets : public QObject
{
    Q_OBJECT
public:
    explicit SerialDataPackets(QObject *parent = nullptr);
    ~SerialDataPackets();

    void start(const QString &portName);   // Open and start reading
    void stop();                           // Close serial port and stop reading
    void setMarkers(char start, char end); // Set start and end markers for packet parsing
    void cleanup();                        // Called externally to stop and cleanup

    float getLatestValue() const;          // Thread-safe getter for latest parsed value

signals:
    void packetReceived(float value);      // Emitted when a valid packet is parsed
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorMessage);

private slots:
    void handleIncomingData();             // Called when new serial data is available
    void parseBuffer();                    // Parse data in circular buffer
    void handleError(QSerialPort::SerialPortError error); // Serial error handling

private:
    void pushToCircularBuffer(uint8_t data);
    QByteArray readFromCircularBuffer();

    QSerialPort serialHandler;
    QByteArray circularBuffer;
    char startMarker = '<';
    char endMarker = '>';

    static constexpr int BUFFER_SIZE = 1024;
    mutable QReadWriteLock valueLock;
    float latestValue = 0.0f;
};

#endif // SERIALDATAPACKETS_H
