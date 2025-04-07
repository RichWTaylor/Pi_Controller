#include "PidController.h"
#include <QDebug>

pidController::pidController(QObject *parent)
    : QObject(parent)
{
    // Initialize Umsdk_wrapper instance (it's already initialized by its constructor)
}

void pidController::onPacketReceived(float value) {
    qDebug() << "Packet received with value:" << value;
    
    lastValue = value; // Always update the last value
    
    if (pidRunning) {
        // Calculate PID output
        double error = setpoint - value;
        integral += error;
        double derivative = error - lastError;
        
        double output = kp * error + ki * integral + kd * derivative;
        
        // Apply the output to control the device
        // For example, if positive output means move up:
        if (output > 0.5) {
            umsdk.moveUp();
        } else if (output < -0.5) {
            umsdk.moveDown();
        }
        
        lastError = error;
        qDebug() << "PID calculation - Error:" << error << "Output:" << output;
    }
}

void pidController::connectToDevice()
{
    umsdk.hello();  // Using the Umsdk_wrapper method to connect to the device
    qDebug() << "Connected to device via Umsdk_wrapper.";
}

void pidController::startPIDControl()
{
    integral = 0.0;
    lastError = 0.0;
    pidRunning = true;
    qDebug() << "PID Control started with setpoint:" << setpoint;
}

void pidController::stopPIDControl()
{
    pidRunning = false;
    qDebug() << "PID Control stopped.";
}

double pidController::getPIDValue()
{
    return lastValue;  // Return the last measured value
}

void pidController::setSetpoint(double target)
{
    setpoint = target;
    qDebug() << "PID setpoint set to:" << setpoint;
}

void pidController::setPIDGains(double p, double i, double d)
{
    kp = p;
    ki = i;
    kd = d;
    qDebug() << "PID gains set to P:" << p << "I:" << i << "D:" << d;
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