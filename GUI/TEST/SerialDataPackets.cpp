#include "SerialDataPackets.h"
#include "SerialWorker.h"
#include <QThread>

SerialDataPackets::SerialDataPackets(QObject *parent)
    : QObject(parent),
      latestValue(0.0)
{
    worker = new SerialWorker();  // Create the worker object
    worker->moveToThread(&workerThread);  // Move worker to worker thread

    connect(worker, &SerialWorker::packetReceived, this, &SerialDataPackets::packetReceived);
    connect(worker, &SerialWorker::errorOccurred, this, &SerialDataPackets::errorOccurred);
}

SerialDataPackets::~SerialDataPackets()
{
    cleanup();
}

void SerialDataPackets::start(const QString &portName)
{
    workerThread.start();
    worker->startReading(portName);
}

void SerialDataPackets::stop()
{
    worker->stopReading();
    workerThread.quit();
    workerThread.wait();
}

void SerialDataPackets::cleanup()
{
    stop();
}

float getLatestValue()
{
    return 0;
}

