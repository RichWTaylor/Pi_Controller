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

    void start(const QString &portName);
    void stop();  // Stop the serial reading thread
    void setMarkers(char start, char end);
    void cleanup();  // Cleanup resources and stop thread

    // Method to get the latest value (read-only, thread-safe)
    float getLatestValue() const;

signals:
    void packetReceived(float value);
    void errorOccurred(const QString &errorMessage);

public slots:
    void handleIncomingData(const QByteArray &data);
    void parseBuffer();

private:
    void pushToCircularBuffer(uint8_t data);
    QByteArray readFromCircularBuffer();

    QSerialPort serialHandler;
    QByteArray circularBuffer;
    char startMarker;
    char endMarker;

    static constexpr int BUFFER_SIZE = 1024;
    mutable QReadWriteLock valueLock;  // Protect the latest value
    float latestValue;  // Store the latest parsed value
};

#endif // SERIALDATAPACKETS_H
