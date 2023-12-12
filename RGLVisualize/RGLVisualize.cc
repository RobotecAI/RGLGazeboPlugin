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

#include <algorithm>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>

#include "RGLVisualize.hh"

namespace rgl
{

/// \brief Private data class for RGLVisualize
class RGLVisualizePrivate
{
  /// \brief Makes a request to populate the scene with markers
  public: void PublishMarkers();

  /// \brief Makes a request to delete all markers related to the point cloud.
  public: void ClearMarkers();

  /// \brief Transport node
  public: gz::transport::Node node;

  /// \brief Name of topic for PointCloudPacked
  public: std::string pointCloudTopic;

  /// \brief List of topics publishing PointCloudPacked.
  public: QStringList pointCloudTopicList;

  /// \brief Protect variables changed from transport and the user
  public: std::recursive_mutex mutex;

  /// \brief Point cloud message containing XYZ positions
  public: gz::msgs::PointCloudPacked pointCloudMsg;

  /// \brief True if showing, changeable at runtime
  public: bool showing{true};
};

/////////////////////////////////////////////////
RGLVisualize::RGLVisualize()
    : gz::gui::Plugin(),
      dataPtr(std::make_unique<RGLVisualizePrivate>())
{
  this->OnRefresh();
}

/////////////////////////////////////////////////
RGLVisualize::~RGLVisualize()
{
  this->dataPtr->ClearMarkers();
}

/////////////////////////////////////////////////
void RGLVisualize::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
  {
    this->title = "RGL Visualize";
  }

  // Parameters from XML
  if (_pluginElem)
  {
    auto pointCloudTopicElem =
        _pluginElem->FirstChildElement("point_cloud_topic");
    if (nullptr != pointCloudTopicElem &&
        nullptr != pointCloudTopicElem->GetText())
    {
      this->SetPointCloudTopicList({pointCloudTopicElem->GetText()});
      this->OnPointCloudTopic(this->dataPtr->pointCloudTopicList.at(0));
    }
  }

  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void RGLVisualize::OnPointCloudTopic(const QString &_pointCloudTopic)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  // Unsubscribe from previous choice
  if (!this->dataPtr->pointCloudTopic.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->pointCloudTopic))
  {
    ignerr << "Unable to unsubscribe from topic ["
           << this->dataPtr->pointCloudTopic << "]" << std::endl;
  }

  // Clear visualization
  this->dataPtr->ClearMarkers();

  this->dataPtr->pointCloudTopic = _pointCloudTopic.toStdString();

  // Request service
  this->dataPtr->node.Request(this->dataPtr->pointCloudTopic,
                              &RGLVisualize::OnPointCloudService, this);

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->pointCloudTopic,
                                     &RGLVisualize::OnPointCloud, this))
  {
    ignerr << "Unable to subscribe to topic ["
            << this->dataPtr->pointCloudTopic << "]\n";
    return;
  }
  ignmsg << "Subscribed to " << this->dataPtr->pointCloudTopic << std::endl;
}

//////////////////////////////////////////////////
void RGLVisualize::Show(bool _show)
{
  this->dataPtr->showing = _show;
  if (_show)
  {
    this->dataPtr->PublishMarkers();
  }
  else
  {
    this->dataPtr->ClearMarkers();
  }
}

/////////////////////////////////////////////////
void RGLVisualize::OnRefresh()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  ignmsg << "Refreshing topic list for point cloud messages." << std::endl;

  // Clear
  this->dataPtr->pointCloudTopicList.clear();

  // Get updated list
  std::vector<std::string> allTopics;
  this->dataPtr->node.TopicList(allTopics);
  for (const auto& topic : allTopics)
  {
    std::vector<gz::transport::MessagePublisher> publishers;
    this->dataPtr->node.TopicInfo(topic, publishers);
    for (const auto& pub : publishers)
    {
      if (pub.MsgTypeName() == "gz.msgs.PointCloudPacked")
      {
        if (std::find(this->dataPtr->pointCloudTopicList.begin(),
                      this->dataPtr->pointCloudTopicList.end(),
                      QString::fromStdString(topic)) != this->dataPtr->pointCloudTopicList.end())
        {
          // already added
          continue;
        }
        this->dataPtr->pointCloudTopicList.push_back(
            QString::fromStdString(topic));
      }
    }
  }
  if (!this->dataPtr->pointCloudTopicList.empty())
  {
    this->OnPointCloudTopic(this->dataPtr->pointCloudTopicList.at(0));
  }

  this->PointCloudTopicListChanged();
}

/////////////////////////////////////////////////
QStringList RGLVisualize::PointCloudTopicList() const
{
  return this->dataPtr->pointCloudTopicList;
}

/////////////////////////////////////////////////
void RGLVisualize::SetPointCloudTopicList(
    const QStringList &_pointCloudTopicList)
{
  this->dataPtr->pointCloudTopicList = _pointCloudTopicList;
  this->PointCloudTopicListChanged();
}

//////////////////////////////////////////////////
void RGLVisualize::OnPointCloud(
    const gz::msgs::PointCloudPacked &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  this->dataPtr->pointCloudMsg = _msg;
  this->dataPtr->PublishMarkers();
}

//////////////////////////////////////////////////
void RGLVisualize::OnPointCloudService(
    const gz::msgs::PointCloudPacked &_msg, bool _result)
{
  if (!_result)
  {
    ignerr << "Service request failed." << std::endl;
    return;
  }
  this->OnPointCloud(_msg);
}

//////////////////////////////////////////////////
void RGLVisualizePrivate::PublishMarkers()
{
  GZ_PROFILE("RGLVisualize::PublishMarkers");

  if (!this->showing)
  {
    return;
  }

  // If point cloud empty, do nothing.
  if (this->pointCloudMsg.height() == 0 &&
      this->pointCloudMsg.width() == 0)
  {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(this->mutex);
  gz::msgs::Marker marker;
  marker.set_ns(this->pointCloudTopic);
  marker.set_id(1);
  marker.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker.set_type(gz::msgs::Marker::POINTS);
  marker.set_visibility(gz::msgs::Marker::GUI);

  gz::msgs::PointCloudPackedIterator<float>
      iterX(this->pointCloudMsg, "x");
  gz::msgs::PointCloudPackedIterator<float>
      iterY(this->pointCloudMsg, "y");
  gz::msgs::PointCloudPackedIterator<float>
      iterZ(this->pointCloudMsg, "z");

  // Index of point in point cloud, visualized or not
  int ptIdx{0};
  auto num_points =
      this->pointCloudMsg.data().size() / this->pointCloudMsg.point_step();
  if (this->pointCloudMsg.data().size() % this->pointCloudMsg.point_step() != 0)
  {
    ignwarn << "Mal-formatted point cloud" << std::endl;
  }

  for (; ptIdx < std::min<int>((int)this->pointCloudMsg.data().size(), (int)num_points);
        ++iterX, ++iterY, ++iterZ, ++ptIdx)
  {
    gz::msgs::Set(marker.add_point(),
                        gz::math::Vector3d(*iterX,
                                                 *iterY,
                                                 *iterZ));
  }

  this->node.Request("/marker", marker);
}

//////////////////////////////////////////////////
void RGLVisualizePrivate::ClearMarkers()
{
  if (this->pointCloudTopic.empty())
  {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(this->mutex);
  gz::msgs::Marker msg;
  msg.set_ns(this->pointCloudTopic);
  msg.set_id(0);
  msg.set_action(gz::msgs::Marker::DELETE_ALL);

  igndbg << "Clearing markers on "
         << this->pointCloudTopic
         << std::endl;

  this->node.Request("/marker", msg);
}

// Register this plugin
GZ_ADD_PLUGIN(RGLVisualize, ::gz::gui::Plugin)

}  // namespace rgl
