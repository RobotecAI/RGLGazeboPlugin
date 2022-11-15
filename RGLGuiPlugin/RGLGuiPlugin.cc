/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 * Copyright 2022 Robotec.AI
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

#include <ignition/msgs/pointcloud_packed.pb.h>

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
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Marker.hh>
#include <ignition/math/Color.hh>

#include "RGLGuiPlugin.hh"

using namespace std::literals::chrono_literals;

/// \brief Private data class for RGLGuiPlugin
class ignition::gui::plugins::RGLGuiPluginPrivate
{
  /// \brief Makes a request to populate the scene with markers
  public: void PublishMarkers();

  /// \brief Makes a request to delete all markers related to the point cloud.
  public: void ClearMarkers();

  /// \brief Transport node
  public: ignition::transport::Node node;

  /// \brief Name of topic for PointCloudPacked
  public: std::string pointCloudTopic;

  /// \brief List of topics publishing PointCloudPacked.
  public: QStringList pointCloudTopicList;

  /// \brief Protect variables changed from transport and the user
  public: std::recursive_mutex mutex;

  /// \brief Point cloud message containing XYZ positions
  public: ignition::msgs::PointCloudPacked pointCloudMsg;

  /// \brief Color for minimum value, changeable at runtime
public: ignition::math::Color minColor{ignition::math::Color::Red};

  /// \brief Size of each point, changeable at runtime
  public: float pointSize{1};

  /// \brief True if showing, changeable at runtime
  public: bool showing{true};

    /// \brief Marks when a new change has been requested.
  public: bool dirty{false};

  public: ignition::rendering::MarkerPtr marker = nullptr;
};

using namespace ignition::gui::plugins;

/////////////////////////////////////////////////
RGLGuiPlugin::RGLGuiPlugin()
  : ignition::gui::Plugin(),
    dataPtr(std::make_unique<RGLGuiPluginPrivate>()) {}

/////////////////////////////////////////////////
RGLGuiPlugin::~RGLGuiPlugin()
{
    if (this->dataPtr->marker != nullptr) {
        this->dataPtr->marker->Destroy();
    }
}

/////////////////////////////////////////////////
void RGLGuiPlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty()) {
      this->title = "rgl Visualize";
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

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void RGLGuiPlugin::OnPointCloudTopic(const QString &_pointCloudTopic)
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
			      &RGLGuiPlugin::OnPointCloudService, this);

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->pointCloudTopic,
				     &RGLGuiPlugin::OnPointCloud, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << this->dataPtr->pointCloudTopic << "]\n";
    return;
  }
  ignmsg << "Subscribed to " << this->dataPtr->pointCloudTopic << std::endl;
}

//////////////////////////////////////////////////
void RGLGuiPlugin::Show(bool _show)
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
void RGLGuiPlugin::OnRefresh()
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
    std::vector<ignition::transport::MessagePublisher> publishers;
    this->dataPtr->node.TopicInfo(topic, publishers);
    for (const auto& pub : publishers)
    {
      if (pub.MsgTypeName() == "ignition.msgs.PointCloudPacked")
      {
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
  this->FloatVTopicListChanged();
}

/////////////////////////////////////////////////
QStringList RGLGuiPlugin::PointCloudTopicList() const
{
  return this->dataPtr->pointCloudTopicList;
}

/////////////////////////////////////////////////
void RGLGuiPlugin::SetPointCloudTopicList(
    const QStringList &_pointCloudTopicList)
{
  this->dataPtr->pointCloudTopicList = _pointCloudTopicList;
  this->PointCloudTopicListChanged();
}

//////////////////////////////////////////////////
void RGLGuiPlugin::OnPointCloud(
    const ignition::msgs::PointCloudPacked &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  this->dataPtr->pointCloudMsg = _msg;
  this->dataPtr->PublishMarkers();
}

//////////////////////////////////////////////////
void RGLGuiPlugin::OnPointCloudService(
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
void RGLGuiPluginPrivate::PublishMarkers()
{
  IGN_PROFILE("RGLGuiPlugin::PublishMarkers");

  if (!this->showing) {
    return;
  }

  this->dirty = true;
}

//////////////////////////////////////////////////
void RGLGuiPluginPrivate::ClearMarkers()
{
  if (this->pointCloudTopic.empty() || marker == nullptr) {
      return;
  }

  marker->ClearPoints();
}

/////////////////////////////////////////////////
QColor RGLGuiPlugin::MinColor() const
{
  return ignition::gui::convert(this->dataPtr->minColor);
}

/////////////////////////////////////////////////
void RGLGuiPlugin::SetMinColor(const QColor &_minColor)
{
  this->dataPtr->minColor = ignition::gui::convert(_minColor);
  this->MinColorChanged();
  this->dataPtr->PublishMarkers();
}

/////////////////////////////////////////////////
float RGLGuiPlugin::PointSize() const
{
  return this->dataPtr->pointSize;
}

/////////////////////////////////////////////////
//! [eventFilter]
bool RGLGuiPlugin::eventFilter(QObject *_obj, QEvent *_event)
{
    if (_event->type() == ignition::gui::events::Render::kType)
    {
        // This event is called in the render thread, so it's safe to make
        // rendering calls here
        this->PerformRenderingOperations();
    }

    // Standard event processing
    return QObject::eventFilter(_obj, _event);
}
//! [eventFilter]

/////////////////////////////////////////////////
void RGLGuiPlugin::SetPointSize(float _pointSize)
{
  this->dataPtr->pointSize = _pointSize;
  this->PointSizeChanged();
  this->dataPtr->PublishMarkers();
}

/////////////////////////////////////////////////
//! [performRenderingOperations]
void RGLGuiPlugin::PerformRenderingOperations()
{
    if (!this->dataPtr->dirty)
    {
        return;
    }

    if (nullptr == this->scene)
    {
        this->FindScene();
    }

    if (nullptr == this->scene) {
        return;
    }

    if (nullptr == this->dataPtr->marker) {
        this->CreateMarker();
    }

    if (nullptr == this->dataPtr->marker) {
        return;
    }

    this->dataPtr->marker->ClearPoints();

    this->dataPtr->marker->SetSize(this->dataPtr->pointSize);

    ignition::rendering::MaterialPtr materialPtr = this->scene->CreateMaterial();
    materialPtr->SetLightingEnabled(false);
    materialPtr->SetDiffuse(this->dataPtr->minColor);
    dataPtr->marker->SetMaterial(materialPtr, true);
    this->scene->DestroyMaterial(materialPtr);

    ignition::msgs::PointCloudPackedIterator<float>
            iterX(this->dataPtr->pointCloudMsg, "x");
    ignition::msgs::PointCloudPackedIterator<float>
            iterY(this->dataPtr->pointCloudMsg, "y");
    ignition::msgs::PointCloudPackedIterator<float>
            iterZ(this->dataPtr->pointCloudMsg, "z");


    auto start_populating_msg = std::chrono::system_clock::now();

    for (; iterX != iterX.End(); ++iterX, ++iterY, ++iterZ)
    {
        // setting the color here doesn't work
        dataPtr->marker->AddPoint(*iterX, *iterY, *iterZ, ignition::math::Color::Red);
    }

    auto end_populating_msg = std::chrono::system_clock::now();
    auto time_to_populate_msg = std::chrono::duration_cast<std::chrono::milliseconds>(end_populating_msg - start_populating_msg);
    ignmsg << "GUI populate message and visualize time: " << time_to_populate_msg.count() << " ms\n";

    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
    this->dataPtr->dirty = false;
}
//! [performRenderingOperations]

void RGLGuiPlugin::CreateMarker() {
    if (nullptr == this->scene) {
        ignerr << "RGLGuiPlugin could not find scene while attempting to create marker";
        return;
    }

    static size_t rgl_id = 0;
    // Create the name for the marker
    std::string name = "__RGL_MARKER_VISUAL__" + std::to_string(rgl_id);

    rgl_id++;

    // Create the new marker
    visualPtr = this->scene->CreateVisual(name);

    // Create and load the marker
    this->dataPtr->marker = this->scene->CreateMarker();

    // Set the marker values from the Marker Message
    dataPtr->marker->SetLayer(0);

    dataPtr->marker->SetLifetime(10min);

    dataPtr->marker->SetSize(this->dataPtr->pointSize);

    dataPtr->marker->SetType(ignition::rendering::MarkerType::MT_POINTS);

    ignition::rendering::MaterialPtr materialPtr = this->scene->CreateMaterial();
    materialPtr->SetLightingEnabled(false);
    materialPtr->SetDiffuse(this->dataPtr->minColor);
    dataPtr->marker->SetMaterial(materialPtr, true);
    this->scene->DestroyMaterial(materialPtr);

    // Add populated marker to the visual
    visualPtr->AddGeometry(dataPtr->marker);

    this->scene->RootVisual()->AddChild(visualPtr);
}

/////////////////////////////////////////////////
void RGLGuiPlugin::FindScene()
{
    auto loadedEngNames = ignition::rendering::loadedEngines();
    if (loadedEngNames.empty())
    {
        igndbg << "No rendering engine is loaded yet" << std::endl;
        return;
    }

    // assume there is only one engine loaded
    auto engineName = loadedEngNames[0];
    if (loadedEngNames.size() > 1)
    {
        igndbg << "More than one engine is available. "
               << "Using engine [" << engineName << "]" << std::endl;
    }
    auto engine = ignition::rendering::engine(engineName);
    if (!engine)
    {
        ignerr << "Internal error: failed to load engine [" << engineName
               << "]. Grid plugin won't work." << std::endl;
        return;
    }

    if (engine->SceneCount() == 0)
    {
        igndbg << "No scene has been created yet" << std::endl;
        return;
    }

    // Get first scene
    auto scenePtr = engine->SceneByIndex(0);
    if (nullptr == scenePtr)
    {
        ignerr << "Internal error: scene is null." << std::endl;
        return;
    }

    if (engine->SceneCount() > 1)
    {
        igndbg << "More than one scene is available. "
               << "Using scene [" << scene->Name() << "]" << std::endl;
    }

    if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
    {
        return;
    }

    this->scene = scenePtr;
}

// Register this plugin
IGNITION_ADD_PLUGIN(RGLGuiPlugin,
		    ignition::gui::Plugin)
