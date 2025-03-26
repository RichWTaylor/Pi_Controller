#ifndef UMSDK_WRAPPER_H
#define UMSDK_WRAPPER_H

#include <QObject>
#include <cstring>
#include <libum.h>

class Umsdk_wrapper : public QObject {
    Q_OBJECT
public:
    explicit Umsdk_wrapper(QObject *parent = nullptr);
    Q_INVOKABLE void moveDown();
    Q_INVOKABLE void moveUp();
    Q_INVOKABLE void moveFwd();
    Q_INVOKABLE void moveBack();
    Q_INVOKABLE void hello();
private:
    LibUm lib;  // Instantiating LibUm object directly (no need for 'new')
};

#endif // UMSDK_WRAPPER_H
