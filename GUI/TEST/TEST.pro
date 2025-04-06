QT += quick serialport

INCLUDEPATH += /home/labadmin/umsdk/inc/
LIBS += -L/home/labadmin/umsdk/build/bin/shared/ -lum
QMAKE_RPATHDIR += /home/labadmin/umsdk/build/bin/shared/

SOURCES += \
    umsdk_wrapper.cpp \
    main.cpp \
    SerialParserWorker.cpp

HEADERS += \
    umsdk_wrapper.h \
    SerialParserWorker.h \
    SerialProtocol.h  # Include shared protocol file

RESOURCES += qml.qrc

QML_IMPORT_PATH = /usr/lib/aarch64-linux-gnu/qt5/qml

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
