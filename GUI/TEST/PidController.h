#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <QObject>
#include "umsdk_wrapper.h"  // Include Umsdk_wrapper class

class pidController : public QObject
{
    Q_OBJECT

public:
    explicit pidController(QObject *parent = nullptr);

    // PID control methods
    Q_SLOT void onPacketReceived(float value);
    
    // Expose UMSDK wrapper functions via helper methods
    Q_INVOKABLE void connectToDevice();
    Q_INVOKABLE void startPIDControl();
    Q_INVOKABLE void stopPIDControl();
    Q_INVOKABLE double getPIDValue();
    
    // New methods for PID control
    Q_INVOKABLE void setSetpoint(double target);
    Q_INVOKABLE void setPIDGains(double p, double i, double d);

    // Access UMSDK wrapper methods through helper functions
    Q_INVOKABLE void moveUp();
    Q_INVOKABLE void moveDown();
    Q_INVOKABLE void moveFwd();
    Q_INVOKABLE void moveBack();

private:
    Umsdk_wrapper umsdk;  // Umsdk_wrapper instance to be owned by pidController
    
    // PID control variables
    double kp = 1.0;  // Proportional gain
    double ki = 0.1;  // Integral gain
    double kd = 0.05; // Derivative gain
    
    double setpoint = 0.0;  // Target value
    double lastError = 0.0; // Previous error
    double integral = 0.0;  // Accumulated error
    double lastValue = 0.0; // Previous input
    bool pidRunning = false;
};

#endif // PIDCONTROLLER_H