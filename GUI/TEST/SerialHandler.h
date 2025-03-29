#pragma once

#include <QObject>
#include <QSerialPort>
#include <QByteArray>
#include <functional>

class SerialHandler : public QObject {
    Q_OBJECT

public:
    explicit SerialHandler(QObject *parent = nullptr);
    ~SerialHandler();

    bool open(const QString &portName, qint32 baudRate = QSerialPort::Baud115200);
    void close();
    bool isOpen() const;

    void setDataCallback(const std::function<void(const QByteArray&)> &callback);

signals:
    void errorOccurred(const QString &error);

private slots:
    void handleReadyRead();

private:
    QSerialPort serial;
    std::function<void(const QByteArray&)> dataCallback;
};
