#ifndef SERIALDATAPACKETS_H
#define SERIALDATAPACKETS_H

#include <QObject>
#include <QByteArray>
#include <QSerialPort>
#include <QReadWriteLock>
#include <QThread>
#include "SerialWorker.h"  // Include SerialWorker definition

class SerialDataPackets : public QObject
{
    Q_OBJECT
    //Q_PROPERTY acts as a bridge between class variables and the Qt property system, 
    //which includes signal-slot connections and QML bindings.

    // Q_PROPERTY is used when you want to expose a a class member as a property to QML
    Q_PROPERTY(float latestValue READ getLatestValue NOTIFY packetReceived) // Expose latestValue to QML
    //Allows latestValue to be accessed in QML.
    // Q_PROPERTY | Declares the property so QML and Qt can recognize it
    // float latestValue | Property name and type
    // READ | getLatestValue	Uses getLatestValue() as the getter function
    // NOTIFY | packetReceived	Updates QML when packetReceived(float value) is emitted

public:
    explicit SerialDataPackets(QObject *parent = nullptr);
    ~SerialDataPackets();

    void start(const QString &portName);   // Open and start reading
    void stop();                           // Close serial port and stop reading
    void setMarkers(char start, char end); // Set start and end markers for packet parsing
    void cleanup();                        // Stop serial & terminate thread

    float getLatestValue() const;          // Thread-safe getter for latest parsed value

signals:
    void packetReceived(float value);      // Emitted when a valid packet is parsed
    void errorOccurred(QSerialPort::SerialPortError error, const QString &errorMessage);

private slots:
    //void handleIncomingData();             // Called when new serial data is available
    //void parseBuffer();                    // Parse data in circular buffer
    //void handleError(QSerialPort::SerialPortError error); // Serial error handling

private:
    void pushToCircularBuffer(uint8_t data);
    QByteArray readFromCircularBuffer();

    SerialWorker *worker;  // Declare the worker pointer
    QThread workerThread;   // Owns a worker thread
    QSerialPort serialHandler;
    QByteArray circularBuffer;
    char startMarker = '<';
    char endMarker = '>';

    static constexpr int BUFFER_SIZE = 1024;
    mutable QReadWriteLock valueLock;
    float latestValue = 0.0f;
};

#endif // SERIALDATAPACKETS_H
