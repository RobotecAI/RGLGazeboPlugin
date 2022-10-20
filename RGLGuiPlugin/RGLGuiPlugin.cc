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
class gz::gui::plugins::RGLGuiPluginPrivate
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
  public: float pointSize{5};

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
    dataPtr(std::make_unique<RGLGuiPluginPrivate>())
{
}

/////////////////////////////////////////////////
RGLGuiPlugin::~RGLGuiPlugin()
{
  this->dataPtr->ClearMarkers();
}

/////////////////////////////////////////////////
void RGLGuiPlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Point cloud";

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

    auto floatVTopicElem =
        _pluginElem->FirstChildElement("float_v_topic");
    if (nullptr != floatVTopicElem &&
        nullptr != floatVTopicElem->GetText())
    {
      this->SetFloatVTopicList({floatVTopicElem->GetText()});
      this->OnFloatVTopic(this->dataPtr->floatVTopicList.at(0));
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
void RGLGuiPlugin::OnFloatVTopic(const QString &_floatVTopic)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  // Unsubscribe from previous choice
  if (!this->dataPtr->floatVTopic.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->floatVTopic))
  {
    ignerr << "Unable to unsubscribe from topic ["
           << this->dataPtr->floatVTopic <<"]" <<std::endl;
  }

  // Clear visualization
  this->dataPtr->ClearMarkers();

  this->dataPtr->floatVTopic = _floatVTopic.toStdString();

  // Request service
  this->dataPtr->node.Request(this->dataPtr->floatVTopic,
			      &RGLGuiPlugin::OnFloatVService, this);

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->floatVTopic,
				     &RGLGuiPlugin::OnFloatV, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << this->dataPtr->floatVTopic << "]\n";
    return;
  }
  ignmsg << "Subscribed to " << this->dataPtr->floatVTopic << std::endl;
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
  this->dataPtr->floatVTopicList.clear();

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
      else if (pub.MsgTypeName() == "ignition.msgs.Float_V")
      {
        this->dataPtr->floatVTopicList.push_back(QString::fromStdString(topic));
      }
    }
  }
  // Handle floats first, so by the time we get the point cloud it can be
  // colored
  if (this->dataPtr->floatVTopicList.size() > 0)
  {
    this->OnFloatVTopic(this->dataPtr->floatVTopicList.at(0));
  }

  if (this->dataPtr->pointCloudTopicList.size() > 0)
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

/////////////////////////////////////////////////
QStringList RGLGuiPlugin::FloatVTopicList() const
{
  return this->dataPtr->floatVTopicList;
}

/////////////////////////////////////////////////
void RGLGuiPlugin::SetFloatVTopicList(
    const QStringList &_floatVTopicList)
{
  this->dataPtr->floatVTopicList = _floatVTopicList;
  this->FloatVTopicListChanged();
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
void RGLGuiPlugin::OnFloatV(const ignition::msgs::Float_V &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  this->dataPtr->floatVMsg = _msg;

  this->dataPtr->minFloatV = std::numeric_limits<float>::max();
  this->dataPtr->maxFloatV = -std::numeric_limits<float>::max();

  for (auto i = 0; i < _msg.data_size(); ++i)
  {
    auto data = _msg.data(i);
    if (data < this->dataPtr->minFloatV)
      this->SetMinFloatV(data);
    if (data > this->dataPtr->maxFloatV)
      this->SetMaxFloatV(data);
  }

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
void RGLGuiPlugin::OnFloatVService(
    const ignition::msgs::Float_V &_msg, bool _result)
{
  if (!_result)
  {
    ignerr << "Service request failed." << std::endl;
    return;
  }
  this->OnFloatV(_msg);
}

//////////////////////////////////////////////////
void RGLGuiPluginPrivate::PublishMarkers()
{
  IGN_PROFILE("RGLGuiPlugin::PublishMarkers");

  if (!this->showing)
    return;

  // If point cloud empty, do nothing.
  if (this->pointCloudMsg.height() == 0 &&
      this->pointCloudMsg.width() == 0)
  {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(this->mutex);
  this->dirty = true;


//  ignition::msgs::Marker marker;
//  marker.set_ns(this->pointCloudTopic + this->floatVTopic);
//  marker.set_id(1);
//  marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
//  marker.set_type(ignition::msgs::Marker::POINTS);
//  marker.set_visibility(ignition::msgs::Marker::GUI);
//
//  ignition::msgs::Set(marker.mutable_scale(),
//    ignition::math::Vector3d::One * this->pointSize);
//
//  ignition::msgs::PointCloudPackedIterator<float>
//      iterX(this->pointCloudMsg, "x");
//  ignition::msgs::PointCloudPackedIterator<float>
//      iterY(this->pointCloudMsg, "y");
//  ignition::msgs::PointCloudPackedIterator<float>
//      iterZ(this->pointCloudMsg, "z");
//
//  // Index of point in point cloud, visualized or not
//  int ptIdx{0};
//  auto minC = this->minColor;
////  auto maxC = this->maxColor;
////  auto floatRange = this->maxFloatV - this->minFloatV;
//
//    auto start_populating_msg = std::chrono::system_clock::now();
//
//  for (; iterX != iterX.End(); ++iterX, ++iterY, ++iterZ, ++ptIdx)
//  {
//    ignition::msgs::Set(marker.add_point(), ignition::math::Vector3d(
//      *iterX,
//      *iterY,
//      *iterZ));
//  }
//
//    auto end_populating_msg = std::chrono::system_clock::now();
//    auto time_to_populate_msg = std::chrono::duration_cast<std::chrono::milliseconds>(end_populating_msg - start_populating_msg);
//    ignmsg << "GUI populate message time: " << time_to_populate_msg.count() << " ms\n";
//
//  auto material = new ignition::msgs::Material(ignition::msgs::Material::default_instance());
//  ignition::msgs::Set(material->mutable_diffuse(), minC);
//  marker.set_allocated_material(material);
//
//  this->node.Request("/marker", marker);
//
//    auto end_req_msg = std::chrono::system_clock::now();
//    auto time_to_req_msg = std::chrono::duration_cast<std::chrono::milliseconds>(end_req_msg - end_populating_msg);
//    ignmsg << "GUI request message time: " << time_to_req_msg.count() << " ms\n";
}

//////////////////////////////////////////////////
void RGLGuiPluginPrivate::ClearMarkers()
{
  if (this->pointCloudTopic.empty() || marker == nullptr)
    return;

  marker->ClearPoints();

//  std::lock_guard<std::recursive_mutex> lock(this->mutex);
//  ignition::msgs::Marker msg;
//  msg.set_id(2137);
//  msg.set_action(ignition::msgs::Marker::DELETE_MARKER);
//
//  igndbg << "Clearing marker on id: 2137" << std::endl;
//
//  this->node.Request("/marker", msg);
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
QColor RGLGuiPlugin::MaxColor() const
{
  return ignition::gui::convert(this->dataPtr->maxColor);
}

/////////////////////////////////////////////////
void RGLGuiPlugin::SetMaxColor(const QColor &_maxColor)
{
  this->dataPtr->maxColor = ignition::gui::convert(_maxColor);
  this->MaxColorChanged();
  this->dataPtr->PublishMarkers();
}

/////////////////////////////////////////////////
float RGLGuiPlugin::MinFloatV() const
{
  return this->dataPtr->minFloatV;
}

/////////////////////////////////////////////////
void RGLGuiPlugin::SetMinFloatV(float _minFloatV)
{
  this->dataPtr->minFloatV = _minFloatV;
  this->MinFloatVChanged();
}

/////////////////////////////////////////////////
float RGLGuiPlugin::MaxFloatV() const
{
  return this->dataPtr->maxFloatV;
}

/////////////////////////////////////////////////
void RGLGuiPlugin::SetMaxFloatV(float _maxFloatV)
{
  this->dataPtr->maxFloatV = _maxFloatV;
  this->MaxFloatVChanged();
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

    if (nullptr == this->scene) return;

    if (nullptr == this->dataPtr->marker) {
        this->CreateMarker();
    }

    if (nullptr == this->dataPtr->marker) return;

    this->dataPtr->marker->ClearPoints();

    ignition::msgs::PointCloudPackedIterator<float>
            iterX(this->dataPtr->pointCloudMsg, "x");
    ignition::msgs::PointCloudPackedIterator<float>
            iterY(this->dataPtr->pointCloudMsg, "y");
    ignition::msgs::PointCloudPackedIterator<float>
            iterZ(this->dataPtr->pointCloudMsg, "z");

//    auto minC = this->dataPtr->minColor;
    auto start_populating_msg = std::chrono::system_clock::now();

    for (; iterX != iterX.End(); ++iterX, ++iterY, ++iterZ)
    {
        // setting the color here doesn't work
        dataPtr->marker->AddPoint(*iterX, *iterY, *iterZ, ignition::math::Color::Green);
    }

    auto end_populating_msg = std::chrono::system_clock::now();
    auto time_to_populate_msg = std::chrono::duration_cast<std::chrono::milliseconds>(end_populating_msg - start_populating_msg);
    ignmsg << "GUI populate message and visualize time: " << time_to_populate_msg.count() << " ms\n";

    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
    this->dataPtr->dirty = false;
}
//! [performRenderingOperations]

void RGLGuiPlugin::CreateMarker() {
    assert(nullptr != this->scene);
    assert(nullptr == this->dataPtr->marker);
    // Create the name for the marker
    std::string name = "__RGL_MARKER_VISUAL_" + std::to_string(2137);

    // Create the new marker
    visualPtr = this->scene->CreateVisual(name);

    // Create and load the marker
    this->dataPtr->marker = this->scene->CreateMarker();

    // Set the marker values from the Marker Message
    dataPtr->marker->SetLayer(0);

    dataPtr->marker->SetLifetime(10min);

    dataPtr->marker->SetType(ignition::rendering::MarkerType::MT_POINTS);

    ignition::rendering::MaterialPtr materialPtr = this->scene->CreateMaterial();
    materialPtr->SetLightingEnabled(false);
    materialPtr->SetDiffuse(ignition::math::Color::Red);
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
