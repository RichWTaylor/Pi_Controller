// SerialParserHandler.cpp
#include "SerialParserHandler.h"
#include "SerialParserWorker.h"
#include <QThread>
#include <QDebug>
#include <QSerialPort>

#include <QMetaType>


SerialParserHandler::SerialParserHandler(QObject *parent) : QObject(parent), serialParserWorker(nullptr)
{
    // Register the QSerialPort::SerialPortError type for signal-slot connections
    qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");


    // Initialize the worker thread and worker
    workerThread = new QThread(this);
    serialParserWorker = new SerialParserWorker();
    serialParserWorker->moveToThread(workerThread);

    // Debug print the portName
    qDebug() << "Port Name in handler:" << portName; // Debugging the portName

    // Connect the thread's started signal to a lambda that calls startReading with the port name
    connect(workerThread, &QThread::started, this, [this]() {
        qDebug() << "Port Name before starting reading:" << portName; // Check before starting reading
        serialParserWorker->startReading(portName);
    });

    connect(serialParserWorker, &SerialParserWorker::packetReceived, this, &SerialParserHandler::onPacketReceived);
    connect(serialParserWorker, &SerialParserWorker::errorOccurred, this, &SerialParserHandler::onErrorOccurred);  // Forward error signal
    connect(workerThread, &QThread::finished, serialParserWorker, &QObject::deleteLater);
    connect(workerThread, &QThread::finished, workerThread, &QObject::deleteLater);

    workerThread->start();
}

SerialParserHandler::~SerialParserHandler() {
    workerThread->quit();
    workerThread->wait();
}

void SerialParserHandler::onPacketReceived(float value) {
    emit packetReceived(value);
}

void SerialParserHandler::onErrorOccurred(QSerialPort::SerialPortError error, const QString &errorString) {
    // Simply forward the error from the worker to the caller (e.g., QML or UI)
    qDebug() << "Error occurred: " << errorString;  // Log the error message if needed
    emit errorOccurred(error, errorString);  // This should be forwarded from here if needed
}

void SerialParserHandler::setPortName(const QString &port) {
    portName = port;
}

// SerialParserHandler.cpp
void SerialParserHandler::startReading(const QString &port) {
    // Make sure portName is set before starting reading
    setPortName(port);  // Optionally, you can call the setter if needed
    qDebug() << "Starting reading on port:" << portName;  // Debug print to confirm

    // Notify the worker to start reading
    serialParserWorker->startReading(portName);
}

