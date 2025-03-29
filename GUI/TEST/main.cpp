#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QThread>
#include "umsdk_wrapper.h"
#include "SerialDataPackets.h"

int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QGuiApplication app(argc, argv);

    Umsdk_wrapper umsdk;  // Main application wrapper

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("umsdk", &umsdk);

    // Create SerialDataPackets in a separate thread
    QThread *serialThread = new QThread();
    SerialDataPackets *serialPackets = new SerialDataPackets();

    // Move serialPackets to the new thread
    serialPackets->moveToThread(serialThread);

    // Start the serial port communication when the thread starts
    QObject::connect(serialThread, &QThread::started, [serialPackets]() {
        serialPackets->setMarkers('<', '>');
        serialPackets->start("/dev/serial0");  // Change to your actual serial port
    });

    // Properly clean up the serial thread on exit
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [serialThread, serialPackets]() {
        serialThread->quit();
        serialThread->wait();
        delete serialPackets;
        delete serialThread;
    });

    // Connect received data to a QML-accessible signal (optional)
    QObject::connect(serialPackets, &SerialDataPackets::packetReceived, [](float value) {
        qDebug() << "Received packet value:" << value;
    });

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    engine.load(url);

    // Start the serial thread
    serialThread->start();

    return app.exec();
}
