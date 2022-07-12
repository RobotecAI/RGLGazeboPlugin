#include <RGLGazeboPlugin.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
	rgl::RGLGazeboPlugin,
	ignition::gazebo::System,
	rgl::RGLGazeboPlugin::ISystemPostUpdate
)

using namespace rgl;

RGLGazeboPlugin::RGLGazeboPlugin() {}

RGLGazeboPlugin::~RGLGazeboPlugin() {}

void RGLGazeboPlugin::PostUpdate(
	const ignition::gazebo::UpdateInfo &_info,
	const ignition::gazebo::EntityComponentManager &_ecm)
{

}