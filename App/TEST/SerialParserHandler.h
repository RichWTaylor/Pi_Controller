#ifndef SERIALPARSERHANDLER_H
#define SERIALPARSERHANDLER_H

#include <QObject>
#include <QThread>
#include "SerialParserWorker.h"

class SerialParserHandler : public QObject
{
    Q_OBJECT
public:
    explicit SerialParserHandler(QObject *parent = nullptr); // Constructor declaration only
    void setPortName(const QString &port); // Function declaration
    ~SerialParserHandler();

    void startReading(const QString &port);

public slots:
    void onPacketReceived(float value);
    void onErrorOccurred(QSerialPort::SerialPortError error, const QString &errorString); // Forwarding error signal

signals:
    void packetReceived(float value);
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorString);

private:
    QThread *workerThread;
    SerialParserWorker *serialParserWorker;
    QString portName; // Declare portName as a private member variable
};

#endif // SERIALPARSERHANDLER_H
