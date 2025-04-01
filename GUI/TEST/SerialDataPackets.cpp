#include "SerialDataPackets.h"
#include "SerialWorker.h"
#include <QThread>
#include <QDebug>

SerialDataPackets::SerialDataPackets(QObject *parent)
    : QObject(parent)
{
    qDebug() << "Creating SerialDataPackets instance...";
    worker = new SerialWorker();  // Create the worker object
    worker->moveToThread(&workerThread);  // Move worker to worker thread

    connect(worker, &SerialWorker::packetReceived, this, &SerialDataPackets::packetReceived);
    connect(worker, &SerialWorker::errorOccurred, this, &SerialDataPackets::errorOccurred);
}

SerialDataPackets::~SerialDataPackets()
{
    qDebug() << "Destroying SerialDataPackets...";
    cleanup();
}

void SerialDataPackets::start(const QString &portName)
{
    qDebug() << "Starting SerialDataPackets with port:" << portName;
    workerThread.start();
    worker->startReading(portName);
}

void SerialDataPackets::stop()
{
    qDebug() << "Stopping SerialDataPackets...";
    worker->stopReading();
    workerThread.quit();
    workerThread.wait();
}

void SerialDataPackets::cleanup()
{
    qDebug() << "Cleaning up SerialDataPackets...";
    stop();
}

float getLatestValue()
{
    qDebug() << "getLatestValue() called!";
    return 0;
}
