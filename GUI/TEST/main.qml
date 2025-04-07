import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "PID Controller with UMSDK Example"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 10

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
            title: "Manual Controls"
            Layout.fillWidth: true
            
            GridLayout {
                columns: 2
                anchors.fill: parent
                
                Item { Layout.fillWidth: true } // Spacer
                
                Button {
                    text: "Up"
                    onClicked: pidController.moveUp()
                }
                
                Item { Layout.fillWidth: true } // Spacer
                
                Button {
                    text: "Down"
                    onClicked: pidController.moveDown()
                }
                
                Button {
                    text: "Forward"
                    onClicked: pidController.moveFwd()
                }
                
                Item { Layout.fillWidth: true } // Spacer
                
                Button {
                    text: "Back"
                    onClicked: pidController.moveBack()
                }
                
                Item { Layout.fillWidth: true } // Spacer
            }
        }
        
        GroupBox {
            title: "PID Controller"
            Layout.fillWidth: true
            
            ColumnLayout {
                anchors.fill: parent
                
                RowLayout {
                    Text { text: "Setpoint:" }
                    TextField {
                        id: setpointField
                        placeholderText: "0.0"
                        validator: DoubleValidator {}
                        Layout.fillWidth: true
                    }
                    Button {
                        text: "Set"
                        onClicked: {
                            pidController.setSetpoint(parseFloat(setpointField.text))
                        }
                    }
                }
                
                GridLayout {
                    columns: 6
                    Layout.fillWidth: true
                    
                    Text { text: "P:" }
                    TextField {
                        id: pGain
                        placeholderText: "1.0"
                        validator: DoubleValidator {}
                        Layout.fillWidth: true
                    }
                    
                    Text { text: "I:" }
                    TextField {
                        id: iGain
                        placeholderText: "0.1"
                        validator: DoubleValidator {}
                        Layout.fillWidth: true
                    }
                    
                    Text { text: "D:" }
                    TextField {
                        id: dGain
                        placeholderText: "0.05"
                        validator: DoubleValidator {}
                        Layout.fillWidth: true
                    }
                }
                
                Button {
                    text: "Update PID Parameters"
                    Layout.fillWidth: true
                    onClicked: {
                        pidController.setPIDGains(
                            parseFloat(pGain.text || "1.0"),
                            parseFloat(iGain.text || "0.1"),
                            parseFloat(dGain.text || "0.05")
                        )
                    }
                }
                
                RowLayout {
                    Layout.fillWidth: true
                    
                    Button {
                        text: "Start PID"
                        Layout.fillWidth: true
                        onClicked: pidController.startPIDControl()
                    }
                    
                    Button {
                        text: "Stop PID"
                        Layout.fillWidth: true
                        onClicked: pidController.stopPIDControl()
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
                
                // Here you could add a chart or graph component
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
    
    // Timer to update display
    Timer {
        interval: 100
        running: true
        repeat: true
        onTriggered: {
            // This forces the Text element showing the current value to update
            // You may need a more sophisticated approach for real-time updating
        }
    }
}