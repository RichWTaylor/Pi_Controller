#pragma once

#include <QObject>
#include "SerialHandler.h"

class SerialDataPackets : public QObject {
    Q_OBJECT

public:
    explicit SerialDataPackets(QObject *parent = nullptr);
    void start(const QString &portName);
    void setMarkers(char start, char end);

signals:
    void packetReceived(float value);
    void errorOccurred(const QString &error);

private:
    SerialHandler serialHandler;
    QByteArray buffer;
    char startMarker = '<';
    char endMarker = '>';

    // Circular buffer size
    static const int BUFFER_SIZE = 128;
    QByteArray circularBuffer;
    int bufferHead = 0;
    int bufferTail = 0;

    // Helper methods for circular buffer
    void pushToCircularBuffer(uint8_t data);
    QByteArray readFromCircularBuffer();

    void handleIncomingData(const QByteArray &data);
    void parseBuffer();
};
