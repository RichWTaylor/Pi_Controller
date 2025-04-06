#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <QObject>
#include <QSerialPort>

class SerialHandler : public QObject
{
    Q_OBJECT
public:
    explicit SerialHandler(QObject *parent = nullptr);

    bool open(const QString &portName, int baudRate = 115200);
    void close();

    void setDataCallback(std::function<void(const QByteArray &)> callback);

signals:
    void errorOccurred(const QString &error);

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort serial;
    std::function<void(const QByteArray &)> dataCallback;
};

#endif // SERIALHANDLER_H
