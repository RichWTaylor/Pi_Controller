#ifndef SERIALDATAPACKETS_H
#define SERIALDATAPACKETS_H

#include <QObject>
#include <QSerialPort>
#include <QThread>
#include "SerialWorker.h"  

class SerialDataPackets : public QObject
{
    Q_OBJECT

public:
    explicit SerialDataPackets(QObject *parent = nullptr);
    ~SerialDataPackets();

    void start(const QString &portName);
    void stop();
    void cleanup();

signals:
    void packetReceived(float value);  // Forward signal from SerialWorker
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorMessage);

private:
    SerialWorker *worker;
    QThread workerThread;
};

#endif // SERIALDATAPACKETS_H
