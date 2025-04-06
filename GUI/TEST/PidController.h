#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <QObject>
#include "umsdk_wrapper.h"  // Include Umsdk_wrapper class

class pidController : public QObject
{
    Q_OBJECT

public:
    explicit pidController(QObject *parent = nullptr);

    // Dummy method for now to handle received packets
    Q_SLOT void onPacketReceived(float value);  // Make sure this is a Q_SLOT

    // Expose UMSDK wrapper functions via helper methods
    Q_INVOKABLE void connectToDevice();
    Q_INVOKABLE void startPIDControl();
    Q_INVOKABLE void stopPIDControl();
    Q_INVOKABLE double getPIDValue();

    // Access UMSDK wrapper methods through helper functions
    Q_INVOKABLE void moveUp();
    Q_INVOKABLE void moveDown();
    Q_INVOKABLE void moveFwd();
    Q_INVOKABLE void moveBack();

private:
    Umsdk_wrapper umsdk;  // Umsdk_wrapper instance to be owned by pidController
};

#endif // PIDCONTROLLER_H
