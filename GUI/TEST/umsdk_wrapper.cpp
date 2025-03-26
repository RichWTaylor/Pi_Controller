#include "umsdk_wrapper.h"
#include <QDebug>

Umsdk_wrapper::Umsdk_wrapper(QObject *parent)
    : QObject(parent) {
    // Initialize lib object directly, no need for 'new'
}


void Umsdk_wrapper::hello() {
    qDebug() << "Launching Sensapex Controller";
    lib.open();
    qDebug() << "Number of devices found " << lib.getDeviceList();

}

void Umsdk_wrapper::moveUp() {
    lib.gotoPos(LIBUM_ARG_UNDEF, LIBUM_ARG_UNDEF, 0, LIBUM_ARG_UNDEF, 10000, LIBUM_USE_LAST_DEV, true, 10);
    qDebug() << "Move Up";   // Use std::cout to print
}

void Umsdk_wrapper::moveDown() {
    lib.gotoPos(LIBUM_ARG_UNDEF, LIBUM_ARG_UNDEF, 25000, LIBUM_ARG_UNDEF, 10000, LIBUM_USE_LAST_DEV, true, 10);
    qDebug() << "Move Down"; 
}

void Umsdk_wrapper::moveFwd() {
    lib.gotoPos(25000, LIBUM_ARG_UNDEF, LIBUM_ARG_UNDEF, LIBUM_ARG_UNDEF, 10000, LIBUM_USE_LAST_DEV, true, 10);
    qDebug() << "Move Up";   // Use std::cout to print
}

void Umsdk_wrapper::moveBack() {
    lib.gotoPos(0, LIBUM_ARG_UNDEF, LIBUM_ARG_UNDEF, LIBUM_ARG_UNDEF, 10000, LIBUM_USE_LAST_DEV, true, 10);
    qDebug() << "Move Down";
}

