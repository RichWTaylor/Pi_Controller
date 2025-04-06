#include "SerialHandler.h"

SerialHandler::SerialHandler(QObject *parent) : QObject(parent) {
    connect(&serial, &QSerialPort::readyRead, this, &SerialHandler::handleReadyRead);
    connect(&serial, &QSerialPort::errorOccurred, this, &SerialHandler::handleError);
}

bool SerialHandler::open(const QString &portName, int baudRate) {
    if (serial.isOpen()) serial.close();

    serial.setPortName(portName);
    serial.setBaudRate(baudRate);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial.open(QIODevice::ReadWrite)) {
        emit errorOccurred("Failed to open port: " + serial.errorString());
        return false;
    }
    return true;
}

void SerialHandler::close() {
    if (serial.isOpen()) {
        serial.close();
    }
}

void SerialHandler::setDataCallback(std::function<void(const QByteArray &)> callback) {
    dataCallback = std::move(callback);
}

void SerialHandler::handleReadyRead() {
    if (dataCallback) {
        QByteArray data = serial.readAll();
        dataCallback(data);
    }
}

void SerialHandler::handleError(QSerialPort::SerialPortError error) {
    if (error != QSerialPort::NoError) {
        emit errorOccurred(serial.errorString());
    }
}
