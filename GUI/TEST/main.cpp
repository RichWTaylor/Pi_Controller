#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QThread>

#include "SerialParserWorker.h"
#include "SerialParserHandler.h"
#include "umsdk_wrapper.h" // Include your umsdk class header

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    Umsdk_wrapper umsdk;

    // Create the SerialParserHandler to manage SerialParserWorker
    SerialParserHandler *serialParserHandler = new SerialParserHandler();

    // Start serial communication after QML is loaded
    serialParserHandler->setPortName("/dev/serial0");  // Set the name only

    // Set up QML engine
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("umsdk", &umsdk);
    engine.rootContext()->setContextProperty("serialParserHandler", serialParserHandler);

    const QUrl url(QStringLiteral("qrc:/main.qml"));

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    engine.load(url);


    return app.exec();
}
