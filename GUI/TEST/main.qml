import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "PID Controller with UMSDK Example"

    ColumnLayout {
        anchors.centerIn: parent

        Button {
            text: "Connect to Device"
            onClicked: {
                pidController.connectToDevice()
            }
        }

        Button {
            text: "Move Up"
            onClicked: {
                pidController.moveUp()  // Call moveUp method in pidController
            }
        }

        Button {
            text: "Move Down"
            onClicked: {
                pidController.moveDown()  // Call moveDown method in pidController
            }
        }

        Button {
            text: "Move Forward"
            onClicked: {
                pidController.moveFwd()  // Call moveFwd method in pidController
            }
        }

        Button {
            text: "Move Back"
            onClicked: {
                pidController.moveBack()  // Call moveBack method in pidController
            }
        }

        Text {
            text: "Current PID Value: " + pidController.getPIDValue()  // Access the PID value from pidController
        }
    }
}
