import QtQuick.Window 2.15
import QtQuick.Controls 2.15

Window {
    width: 500
    height: 480
    visible: true
    title: qsTr("Sensapex Controller v1.0")


    Button {
        text: "DOWN"
        x: 200
        y: 300
        onClicked: umsdk.moveDown()  // Call moveDown() from the Umsdk_wrapper instance
    }

    Button {
        text: "UP"       
        x: 200
         y: 100
        onClicked: umsdk.moveUp()  // Call moveUp() from the Umsdk_wrapper instance
    }


    Button {
        text: "BACK"
        x: 50
        y: 200

        onClicked: umsdk.moveBack()  // Call moveUp() from the Umsdk_wrapper instance
    }

    Button {
        text: "FWD"
        x: 350
        y: 200
        onClicked: umsdk.moveFwd()  // Call moveDown() from the Umsdk_wrapper instance
    }
}
