#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "SerialParserWorker.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    // Create and move SerialParserWorker to its own thread
    QThread *workerThread = new QThread;
    SerialParserWorker *serialWorker = new SerialParserWorker();
    serialWorker->moveToThread(workerThread);

    // Clean up thread and worker on exit
    QObject::connect(workerThread, &QThread::finished, serialWorker, &QObject::deleteLater);
    QObject::connect(&app, &QCoreApplication::aboutToQuit, workerThread, &QThread::quit);
    QObject::connect(workerThread, &QThread::finished, workerThread, &QObject::deleteLater);

    // Start the serial thread
    workerThread->start();

    // Optional: start serial communication right away
    QMetaObject::invokeMethod(serialWorker, "startReading", Qt::QueuedConnection,
                              Q_ARG(QString, "ttyAMA0")); // or any port name you use

    // Set up QML engine
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("serialWorker", serialWorker);
    const QUrl url(QStringLiteral("qrc:/main.qml"));

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    engine.load(url);

    return app.exec();
}
