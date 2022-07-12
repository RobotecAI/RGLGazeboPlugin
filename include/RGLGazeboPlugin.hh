#pragma once

#include <ignition/gazebo/System.hh>


namespace rgl {

struct RGLGazeboPlugin : public ignition::gazebo::System, public ignition::gazebo::ISystemPostUpdate
{
	RGLGazeboPlugin();
	~RGLGazeboPlugin() override;
	void PostUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm) override;
};

}