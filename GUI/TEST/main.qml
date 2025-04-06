import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick 2.15

Window {
    width: 500
    height: 480
    visible: true
    title: qsTr("Sensapex Controller v1.0")

    property real receivedValue: 0.0  // Holds latest received value

    Connections {
        target: serialWorker  // changed from serialPackets
        function onPacketReceived(value) {
            if (value !== value) {
                console.warn("Ignored NAN value");
            } else {
                receivedValue = value;
            }
        }
    }

    Button {
        text: "DOWN"
        x: 200
        y: 300
        onClicked: umsdk.moveDown()
    }

    Button {
        text: "UP"
        x: 200
        y: 100
        onClicked: umsdk.moveUp()
    }

    Button {
        text: "BACK"
        x: 50
        y: 200
        onClicked: umsdk.moveBack()
    }

    Button {
        text: "FWD"
        x: 350
        y: 200
        onClicked: umsdk.moveFwd()
    }

    Text {
        text: "Received Value: " + receivedValue
        x: 200
        y: 250
    }
}
