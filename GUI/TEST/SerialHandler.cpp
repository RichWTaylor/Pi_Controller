#include "SerialHandler.hpp"

SerialHandler::SerialHandler(QObject *parent) : QObject(parent) {
    connect(&serial, &QSerialPort::readyRead, this, &SerialHandler::handleReadyRead);
    connect(&serial, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error != QSerialPort::NoError)
            emit errorOccurred(serial.errorString());
    });
}

SerialHandler::~SerialHandler() {
    close();
}

bool SerialHandler::open(const QString &portName, qint32 baudRate) {
    serial.setPortName(portName);
    serial.setBaudRate(baudRate);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    return serial.open(QIODevice::ReadWrite);
}

void SerialHandler::close() {
    if (serial.isOpen())
        serial.close();
}

bool SerialHandler::isOpen() const {
    return serial.isOpen();
}

void SerialHandler::setDataCallback(const std::function<void(const QByteArray&)> &callback) {
    dataCallback = callback;
}

void SerialHandler::handleReadyRead() {
    QByteArray data = serial.readAll();
    if (dataCallback)
        dataCallback(data);
}
