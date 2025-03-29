QT += quick serialport  # Add serialport module for QSerialPort

INCLUDEPATH += /home/labadmin/umsdk/inc/

LIBS += -L/home/labadmin/umsdk/build/bin/shared/ -lum # for libum!

QMAKE_RPATHDIR += /home/labadmin/umsdk/build/bin/shared/

SOURCES += \
        umsdk_wrapper.cpp \
        main.cpp \
        SerialHandler.cpp \
        SerialDataPackets.cpp

HEADERS += \
        umsdk_wrapper.h \
        SerialHandler.h \
        SerialDataPackets.h

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH = /usr/lib/aarch64-linux-gnu/qt5/qml

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
