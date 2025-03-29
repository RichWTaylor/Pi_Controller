#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
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

    // Move the serialPackets object to the new thread
    serialPackets->moveToThread(serialThread);

    // When the thread starts, initialize serial communication
    QObject::connect(serialThread, &QThread::started, [serialPackets]() {
        serialPackets->setMarkers('<', '>');
        serialPackets->start("/dev/serial0");  // Change to your actual serial port
    });

    // Clean up when the application quits
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [serialThread, serialPackets]() {
        serialPackets->cleanup();  // Stop serial communication and clean up
        serialThread->quit();
        serialThread->wait();  // Ensure the thread finishes before continuing
        delete serialPackets;
        delete serialThread;
    });

    // Connect received data to a QML-accessible signal
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
