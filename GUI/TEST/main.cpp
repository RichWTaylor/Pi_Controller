#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "umsdk_wrapper.h"
#include "SerialDataPackets.h"
#include <QMetaType>
#include <QDebug>

int main(int argc, char *argv[]) {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    qDebug() << "Starting application...";

    // Register QSerialPort::SerialPortError type
    qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");

    QGuiApplication app(argc, argv);

    Umsdk_wrapper umsdk;
    SerialDataPackets serialPackets;  // This will now manage the worker thread

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("umsdk", &umsdk);
    engine.rootContext()->setContextProperty("serialPackets", &serialPackets); // Now, serialPackets is available in QML.

    qDebug() << "Initializing SerialDataPackets...";

    // Start serial communication
    serialPackets.start("/dev/serial0");  // Start the serial communication in the worker thread

    // Connect received data to QML (QML qDebug)
    QObject::connect(&serialPackets, &SerialDataPackets::packetReceived, [](float value) {
        qDebug() << "(main.cpp) Received packet value:" << value;
    });

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl) {
            qCritical() << "Failed to load QML.";
            QCoreApplication::exit(-1);
        }
    }, Qt::QueuedConnection);

    engine.load(url);

    QObject::connect(&engine, &QQmlApplicationEngine::warnings,
                     [](const QList<QQmlError> &warnings) {
        for (const auto &warning : warnings)
            qWarning() << "QML Warning:" << warning.toString();
    });

    qDebug() << "Application initialized successfully.";

    return app.exec();
}
