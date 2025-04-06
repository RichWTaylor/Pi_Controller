#include "PidController.h"
#include <QDebug>

pidController::pidController(QObject *parent)
    : QObject(parent)
{
    // Initialize Umsdk_wrapper instance (it’s already initialized by its constructor)
}

void onPacketReceived(float value) {
    // Dummy processing: just print the received value for now
    qDebug() << "Packet received with value:" << value;

    // Here you can implement actual PID control logic or pass the value to other parts of your controller
}

void pidController::connectToDevice()
{
    umsdk.hello();  // Using the Umsdk_wrapper method to connect to the device
    qDebug() << "Connected to device via Umsdk_wrapper.";
}

void pidController::startPIDControl()
{
    // Start PID control logic here
    qDebug() << "PID Control started.";
}

void pidController::stopPIDControl()
{
    // Stop PID control logic here
    qDebug() << "PID Control stopped.";
}

double pidController::getPIDValue()
{
    // Implement logic to fetch PID value
    return 42.0;  // Placeholder value
}

void pidController::moveUp()
{
    umsdk.moveUp();  // Call Umsdk_wrapper's moveUp
}

void pidController::moveDown()
{
    umsdk.moveDown();  // Call Umsdk_wrapper's moveDown
}

void pidController::moveFwd()
{
    umsdk.moveFwd();  // Call Umsdk_wrapper's moveFwd
}

void pidController::moveBack()
{
    umsdk.moveBack();  // Call Umsdk_wrapper's moveBack
}
