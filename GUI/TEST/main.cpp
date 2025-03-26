#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "umsdk_wrapper.h"

int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QGuiApplication app(argc, argv);

    Umsdk_wrapper umsdk;  // Create a single instance

    QQmlApplicationEngine engine;

    // Register the Umsdk_wrapper instance to QML
    engine.rootContext()->setContextProperty("umsdk", &umsdk);

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    engine.load(url);

    // You can now call C++ functions here
    umsdk.hello();  // Example call to initialize hardware

    return app.exec();
}
