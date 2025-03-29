#ifndef SERIALDATAPACKETS_H
#define SERIALDATAPACKETS_H

#include <QObject>
#include "SerialHandler.h"

class SerialDataPackets : public QObject
{
    Q_OBJECT
public:
    explicit SerialDataPackets(QObject *parent = nullptr);
    ~SerialDataPackets();  // Destructor declaration

    void start(const QString &portName);
    void stop();
    void setMarkers(char start, char end);

signals:
    void packetReceived(float value);
    void errorOccurred(const QString &error);

private:
    static constexpr int BUFFER_SIZE = 1024;
    uint8_t circularBuffer[BUFFER_SIZE];
    int head = 0;
    int tail = 0;

    char startMarker = '<';
    char endMarker = '>';

    SerialHandler serialHandler;

    void pushToCircularBuffer(uint8_t data);
    bool popFromCircularBuffer(uint8_t &data);
    bool isCircularBufferEmpty() const;
    int circularBufferCount() const;
    void parseBuffer();
    void handleIncomingData(const QByteArray &data);
};

#endif // SERIALDATAPACKETS_H
