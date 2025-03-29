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

    // Create SerialDataPackets directly in the main thread
    SerialDataPackets serialPackets;
    serialPackets.setMarkers('<', '>');
    serialPackets.start("/dev/serial0");  // Change this to your actual serial port

    // Connect received data to a QML-accessible signal (optional)
    QObject::connect(&serialPackets, &SerialDataPackets::packetReceived, [](float value) {
        qDebug() << "Received packet value:" << value;
    });

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    engine.load(url);

    return app.exec();
}
