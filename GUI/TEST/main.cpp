#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QThread>

#include "SerialParserWorker.h"
#include "SerialParserHandler.h"
#include "umsdk_wrapper.h" // Include your umsdk class header
#include "PidController.h" // Include the pidController class header

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    // Create the umsdk wrapper to move stage
    //Umsdk_wrapper umsdk;

    // Create the SerialParserHandler to manage SerialParserWorker
    SerialParserHandler *serialParserHandler = new SerialParserHandler();
    serialParserHandler->setPortName("/dev/serial0");  // Set the name only

    // PID Controller
    pidController pidCtrl;

    // Connect the signal to the pidController slot
    QObject::connect(serialParserHandler, &SerialParserHandler::packetReceived, &pidCtrl, &pidController::onPacketReceived);

    // Set up QML engine
    QQmlApplicationEngine engine;

    // Expose instances to QML
    //engine.rootContext()->setContextProperty("umsdk", &umsdk);  // Expose umsdk to QML
    engine.rootContext()->setContextProperty("serialParserHandler", serialParserHandler);  // Expose serialParserHandler to QML
    engine.rootContext()->setContextProperty("pidController", &pidCtrl);  // Expose pidCtrl to QML

    const QUrl url(QStringLiteral("qrc:/main.qml"));

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    engine.load(url);

    return app.exec();
}
