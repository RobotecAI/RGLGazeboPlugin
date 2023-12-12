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

#ifndef RGL_RGLVISUALIZE_HH_
#define RGL_RGLVISUALIZE_HH_

#include <memory>

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/gui/Plugin.hh>

namespace rgl
{

class RGLVisualizePrivate;

/// \brief Visualize `gz::msgs::PointCloudPacked` messages in a 3D
/// scene.
///
/// This is a port of PointCloud gui plugin from Gazebo Garden to enable non-uniform
/// point clouds visualization in Fortress that are produced by RGLServerPlugin.
/// https://github.com/gazebosim/gz-gui/tree/gz-gui7/src/plugins/point_cloud
/// When using RGLServerPlugin with the newer Gazebo release than Fortress,
/// RGLVisualize plugin can be removed.
///
/// Parameters:
/// * `<point_cloud_topic>`: Topic to receive
///      `gz::msgs::PointCloudPacked` messages.
class RGLVisualize : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief List of topics publishing PointCloudPacked messages
  Q_PROPERTY(
    QStringList pointCloudTopicList
    READ PointCloudTopicList
    WRITE SetPointCloudTopicList
    NOTIFY PointCloudTopicListChanged
  )

  /// \brief Constructor
  public: RGLVisualize();

  /// \brief Destructor
  public: ~RGLVisualize() override;

  // Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Callback function for point cloud topic.
  /// \param[in] _msg Point cloud message
  public: void OnPointCloud(const gz::msgs::PointCloudPacked &_msg);

  /// \brief Callback function for point cloud service
  /// \param[in] _msg Point cloud message
  /// \param[out] _result True on success.
  public: void OnPointCloudService(const gz::msgs::PointCloudPacked &_msg,
                                   bool _result);

  /// \brief Get the topic list
  /// \return List of topics
  public: Q_INVOKABLE QStringList PointCloudTopicList() const;

  /// \brief Set the topic list from a string
  /// \param[in] _pointCloudTopicList List of topics.
  public: Q_INVOKABLE void SetPointCloudTopicList(
      const QStringList &_pointCloudTopicList);

  /// \brief Notify that topic list has changed
  signals: void PointCloudTopicListChanged();

  /// \brief Set topic to subscribe to for point cloud.
  /// \param[in] _topicName Name of selected topic
  public: Q_INVOKABLE void OnPointCloudTopic(const QString &_topicName);

  /// \brief Set whether to show the point cloud.
  /// \param[in] _show Boolean value for displaying the points.
  public: Q_INVOKABLE void Show(bool _show);

  /// \brief Callback when refresh button is pressed.
  public: Q_INVOKABLE void OnRefresh();

  /// \internal
  /// \brief Pointer to private data
  private: std::unique_ptr<RGLVisualizePrivate> dataPtr;
};

}  // namespace rgl

#endif
