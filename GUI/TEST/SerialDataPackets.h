#pragma once

#include <QObject>
#include "SerialHandler.hpp"

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

    void handleIncomingData(const QByteArray &data);
    void parseBuffer();
};
