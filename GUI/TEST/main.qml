import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {

    Connections {
        target: serialParserWorker
        onPacketReceived: (value) => {
            serialValueText.text = "Latest Serial Value: " + value.toFixed(3)
        }
    }

    visible: true
    width: 640
    height: 1080
    title: "PID Controller with UMSDK Example"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 10

        GroupBox {
            title: "Serial Data Monitor"
            Layout.fillWidth: true

            ColumnLayout {
                anchors.fill: parent

                Text {
                    id: serialValueText
                    text: "" 
                    font.pixelSize: 16
                }
            }
        }

        GroupBox {
            title: "Device Connection"
            Layout.fillWidth: true
            
            ColumnLayout {
                anchors.fill: parent
                
                Button {
                    text: "Connect to Device"
                    Layout.fillWidth: true
                    onClicked: {
                        pidController.connectToDevice()
                    }
                }
            }
        }
      

        GroupBox {
            title: "Data Monitor"
            Layout.fillWidth: true
            Layout.fillHeight: true

            ColumnLayout {
                anchors.fill: parent

                Text {
                    text: "Current Value: " + pidController.getPIDValue().toFixed(2)
                    font.pixelSize: 16
                    Layout.alignment: Qt.AlignCenter
                }

                Rectangle {
                    color: "lightgray"
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    Text {
                        anchors.centerIn: parent
                        text: "Data visualization could go here"
                        color: "gray"
                    }
                }
            }
        }

        Item {
            Layout.fillHeight: true
        }
    }

}
