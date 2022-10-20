/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import ignition.gui 1.0
import "qrc:/qml"

ColumnLayout {
  spacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 350
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  function isUniform() {
    return RGLGuiPlugin.minFloatV >= RGLGuiPlugin.maxFloatV
  }

  RowLayout {
    spacing: 10
    Layout.fillWidth: true

    Switch {
      Layout.alignment: Qt.AlignHCenter
      id: displayVisual
      Layout.columnSpan: 5
      Layout.fillWidth: true
      text: qsTr("Show")
      checked: true
      onToggled: {
        RGLGuiPlugin.Show(checked)
      }
    }

    RoundButton {
      Layout.columnSpan: 1
      text: "\u21bb"
      Material.background: Material.primary
      onClicked: {
        RGLGuiPlugin.OnRefresh();
        pcCombo.currentIndex = 0
        // floatCombo.currentIndex = 0
      }
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Refresh list of topics publishing point clouds and float vectors")
    }
  }

  GridLayout {
    columns: 3
    columnSpacing: 10
    Layout.fillWidth: true

    Label {
      Layout.columnSpan: 1
      text: "Point cloud"
    }

    ComboBox {
      Layout.columnSpan: 2
      id: pcCombo
      Layout.fillWidth: true
      model: RGLGuiPlugin.RGLGuiPluginTopicList
      currentIndex: 0
      onCurrentIndexChanged: {
        if (currentIndex < 0)
          return;
        RGLGuiPlugin.OnRGLGuiPluginTopic(textAt(currentIndex));
      }
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Gazebo Transport topics publishing RGLGuiPluginPacked messages")
    }

    Label {
      Layout.columnSpan: 1
      text: "Float vector"
    }

    ComboBox {
      Layout.columnSpan: 2
      id: floatCombo
      Layout.fillWidth: true
      model: RGLGuiPlugin.floatVTopicList
      currentIndex: 0
      onCurrentIndexChanged: {
        if (currentIndex < 0)
          return;
        RGLGuiPlugin.OnFloatVTopic(textAt(currentIndex));
      }
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Gazebo Transport topics publishing FloatV messages, used to color each point on the cloud")
    }

    Label {
      Layout.columnSpan: 1
      text: "Point size"
    }

    IgnSpinBox {
      id: pointSizeSpin
      value: RGLGuiPlugin.pointSize
      minimumValue: 1
      maximumValue: 1000
      decimals: 0
      onEditingFinished: {
        RGLGuiPlugin.SetPointSize(pointSizeSpin.value)
      }
    }
  }

  RowLayout {
    spacing: 10
    Layout.fillWidth: true

    Label {
      Layout.columnSpan: 1
      text: isUniform() ? "Color" : "Min"
    }

    Label {
      Layout.columnSpan: 1
      Layout.maximumWidth: 50
      text: RGLGuiPlugin.minFloatV.toFixed(4)
      elide: Text.ElideRight
      visible: !isUniform()
    }

    Button {
      Layout.columnSpan: 1
      id: minColorButton
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Color for minimum value")
      onClicked: minColorDialog.open()
      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        radius: 5
        border.color: RGLGuiPlugin.minColor
        border.width: 2
        color: RGLGuiPlugin.minColor
      }
      ColorDialog {
        id: minColorDialog
        title: "Choose a color for the minimum value"
        visible: false
        onAccepted: {
          RGLGuiPlugin.SetMinColor(minColorDialog.color)
          minColorDialog.close()
        }
        onRejected: {
          minColorDialog.close()
        }
      }
    }

    Button {
      Layout.columnSpan: 1
      id: maxColorButton
      visible: !isUniform()
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Color for maximum value")
      onClicked: maxColorDialog.open()
      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        radius: 5
        border.color: RGLGuiPlugin.maxColor
        border.width: 2
        color: RGLGuiPlugin.maxColor
      }
      ColorDialog {
        id: maxColorDialog
        title: "Choose a color for the maximum value"
        visible: false
        onAccepted: {
          RGLGuiPlugin.SetMaxColor(maxColorDialog.color)
          maxColorDialog.close()
        }
        onRejected: {
          maxColorDialog.close()
        }
      }
    }

    Label {
      Layout.columnSpan: 1
      Layout.maximumWidth: 50
      text: RGLGuiPlugin.maxFloatV.toFixed(4)
      elide: Text.ElideRight
      visible: !isUniform()
    }

    Label {
      Layout.columnSpan: 1
      text: "Max"
      visible: !isUniform()
    }
  }

  Item {
    Layout.columnSpan: 6
    width: 10
    Layout.fillHeight: true
  }
}
