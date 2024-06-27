/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 * Modifications copyright (C) 2022 Robotec.AI
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
import gz.gui 1.0
import "qrc:/qml"

ColumnLayout {
  spacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 250
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

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
        RGLVisualize.Show(checked)
      }
    }

    RoundButton {
      Layout.columnSpan: 1
      text: "\u21bb"
      Material.background: Material.primary
      onClicked: {
        RGLVisualize.OnRefresh();
        pcCombo.currentIndex = 0
        floatCombo.currentIndex = 0
      }
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Refresh list of topics publishing point clouds")
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
      model: RGLVisualize.pointCloudTopicList
      currentIndex: 0
      onCurrentIndexChanged: {
        if (currentIndex < 0)
          return;
        RGLVisualize.OnPointCloudTopic(textAt(currentIndex));
      }
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      ToolTip.text: qsTr("Gazebo Transport topics publishing PointCloudPacked messages")
    }
  }
}
