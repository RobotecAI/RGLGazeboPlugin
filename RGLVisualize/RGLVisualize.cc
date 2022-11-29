/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 * Copyright (C) 2022 Robotec.AI
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

#include "ignition/msgs/pointcloud_packed.pb.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/PointCloudPackedUtils.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

#include "RGLVisualize.hh"

/// \brief Private data class for RGLVisualize
class ignition::gui::plugins::RGLVisualizePrivate
{
    /// \brief Makes a request to populate the scene with markers
public: void PublishMarkers();

    /// \brief Makes a request to delete all markers related to the point cloud.
public: void ClearMarkers();

    /// \brief Transport node
public: ignition::transport::Node node;

    /// \brief Name of topic for PointCloudPacked
public: std::string pointCloudTopic{""};

    /// \brief Name of topic for FloatV
public: std::string floatVTopic{""};

    /// \brief List of topics publishing PointCloudPacked.
public: QStringList pointCloudTopicList;

    /// \brief List of topics publishing FloatV.
public: QStringList floatVTopicList;

    /// \brief Protect variables changed from transport and the user
public: std::recursive_mutex mutex;

    /// \brief Point cloud message containing XYZ positions
public: ignition::msgs::PointCloudPacked pointCloudMsg;

    /// \brief Message holding a float vector.
public: ignition::msgs::Float_V floatVMsg;

    /// \brief Minimum value in latest float vector
public: float minFloatV{std::numeric_limits<float>::max()};

    /// \brief Maximum value in latest float vector
public: float maxFloatV{-std::numeric_limits<float>::max()};

    /// \brief Color for minimum value, changeable at runtime
public: ignition::math::Color minColor{1.0f, 0.0f, 0.0f, 1.0f};

    /// \brief Color for maximum value, changeable at runtime
public: ignition::math::Color maxColor{0.0f, 1.0f, 0.0f, 1.0f};

    /// \brief Size of each point, changeable at runtime
public: float pointSize{1};

    /// \brief True if showing, changeable at runtime
public: bool showing{true};
};

using namespace ignition;
using namespace gui;
using namespace plugins;

/////////////////////////////////////////////////
RGLVisualize::RGLVisualize()
        : ignition::gui::Plugin(),
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
        this->title = "RGL Visualize";

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

    ignition::gui::App()->findChild<
            ignition::gui::MainWindow *>()->installEventFilter(this);
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
              << this->dataPtr->pointCloudTopic <<"]" <<std::endl;
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
    for (auto topic : allTopics)
    {
        std::vector<ignition::transport::MessagePublisher> publishers;
        this->dataPtr->node.TopicInfo(topic, publishers);
        for (auto pub : publishers)
        {
            if (pub.MsgTypeName() == "ignition.msgs.PointCloudPacked")
            {
                this->dataPtr->pointCloudTopicList.push_back(
                        QString::fromStdString(topic));
            }
        }
    }
    if (this->dataPtr->pointCloudTopicList.size() > 0)
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
        const ignition::msgs::PointCloudPacked &_msg)
{
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
    this->dataPtr->pointCloudMsg = _msg;
    this->dataPtr->PublishMarkers();
}

//////////////////////////////////////////////////
void RGLVisualize::OnPointCloudService(
        const ignition::msgs::PointCloudPacked &_msg, bool _result)
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
    IGN_PROFILE("RGLVisualize::PublishMarkers");

    if (!this->showing)
        return;

    // If point cloud empty, do nothing.
    if (this->pointCloudMsg.height() == 0 &&
        this->pointCloudMsg.width() == 0)
    {
        return;
    }

    std::lock_guard<std::recursive_mutex> lock(this->mutex);
    ignition::msgs::Marker marker;
    marker.set_ns(this->pointCloudTopic);
    marker.set_id(1);
    marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    marker.set_type(ignition::msgs::Marker::POINTS);
    marker.set_visibility(ignition::msgs::Marker::GUI);

    ignition::msgs::Set(marker.mutable_scale(),
                  ignition::math::Vector3d::One * this->pointSize);

    ignition::msgs::PointCloudPackedIterator<float>
            iterX(this->pointCloudMsg, "x");
    ignition::msgs::PointCloudPackedIterator<float>
            iterY(this->pointCloudMsg, "y");
    ignition::msgs::PointCloudPackedIterator<float>
            iterZ(this->pointCloudMsg, "z");

    // Index of point in point cloud, visualized or not
    int ptIdx{0};
    auto num_points =
            this->pointCloudMsg.data().size() / this->pointCloudMsg.point_step();
    if (this->pointCloudMsg.data().size() % this->pointCloudMsg.point_step() != 0)
    {
        ignwarn << "Mal-formatted pointcloud" << std::endl;
    }

    for (; ptIdx < std::min<int>((int)this->pointCloudMsg.data().size(), (int)num_points);
           ++iterX, ++iterY, ++iterZ, ++ptIdx)
    {
        ignition::msgs::Set(marker.add_point(), ignition::math::Vector3d(
                *iterX,
                *iterY,
                *iterZ));
    }

    this->node.Request("/marker", marker);
}

//////////////////////////////////////////////////
void RGLVisualizePrivate::ClearMarkers()
{
    if (this->pointCloudTopic.empty())
        return;

    std::lock_guard<std::recursive_mutex> lock(this->mutex);
    ignition::msgs::Marker msg;
    msg.set_ns(this->pointCloudTopic + this->floatVTopic);
    msg.set_id(0);
    msg.set_action(ignition::msgs::Marker::DELETE_ALL);

    igndbg << "Clearing markers on "
          << this->pointCloudTopic
          << std::endl;

    this->node.Request("/marker", msg);
}

/////////////////////////////////////////////////
float RGLVisualize::PointSize() const
{
    return this->dataPtr->pointSize;
}

/////////////////////////////////////////////////
void RGLVisualize::SetPointSize(float _pointSize)
{
    this->dataPtr->pointSize = _pointSize;
    this->PointSizeChanged();
    this->dataPtr->PublishMarkers();
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gui::plugins::RGLVisualize,
        ignition::gui::Plugin)