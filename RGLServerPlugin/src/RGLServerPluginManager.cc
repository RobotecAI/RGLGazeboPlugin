// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <ignition/plugin/Register.hh>

#include "RGLServerPluginManager.hh"

#define PARAM_IGNORE_LINKS_ID "ignore_links"

IGNITION_ADD_PLUGIN(
    rgl::RGLServerPluginManager,
    ignition::gazebo::System,
    rgl::RGLServerPluginManager::ISystemConfigure,
    rgl::RGLServerPluginManager::ISystemPostUpdate
)

using namespace std::placeholders;

namespace rgl
{

void RGLServerPluginManager::Configure(
        const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm,
        ignition::gazebo::EventManager& evm)
{
    ValidateRGLVersion();
    if (!CheckRGL(rgl_configure_logging(RGL_LOG_LEVEL_ERROR, nullptr, true))) {
        ignerr << "Failed to configure RGL logging.\n";
    }

    if (sdf->HasElement(PARAM_IGNORE_LINKS_ID)) {
        auto ignoreLinksStr = sdf->Get<std::string>(PARAM_IGNORE_LINKS_ID);

        std::istringstream iss(ignoreLinksStr);
        std::copy(std::istream_iterator<std::string>(iss),
                  std::istream_iterator<std::string>(),
                  inserter(linksToIgnore, linksToIgnore.begin()));
    }

    for (auto&& s : linksToIgnore) {
        ignerr << "ignore link: " << s << "\n";
    }
}

void RGLServerPluginManager::PostUpdate(
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm)
{
//    // Debug: Print name of the entity
//    ecm.EachNew<ignition::gazebo::components::Name>(
//            [](const ignition::gazebo::Entity &_entity,
//               const ignition::gazebo::components::Name *_name) -> bool
//            {
//                std::cout << "New entity name: " << _name->Data() << std::endl;
//                return true;
//            });

    ecm.EachNew<ignition::gazebo::components::Link>(
            [&](const ignition::gazebo::Entity& entity,
                const ignition::gazebo::components::Link* link) {
                auto linkName = ecm.Component<ignition::gazebo::components::Name>(entity);
                if (linkName) {
                    ignerr << "new link: " << linkName->Data() << "\n";
                    if (linksToIgnore.contains(linkName->Data())) {
                        for (auto descendant: ecm.Descendants(entity)) {
                            ignerr << "entity ignored: " << descendant << " in link " << linkName->Data() << "\n";
                            entitiesToIgnore.insert(descendant);
                        }
                    }
                }
                return true;
            });

    ecm.EachNew<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            ([this, &ecm](auto&& entity, auto&& visual, auto&& geometry) {
                return LoadEntityToRGLCb(entity, visual, geometry, ecm);
            });

    ecm.EachNew<ignition::gazebo::components::LaserRetro>
            ([this](auto&& entity, auto&& laserRetro) {
                return SetLaserRetroCb(entity, laserRetro);
            });

    ecm.EachRemoved<ignition::gazebo::components::Visual, ignition::gazebo::components::Geometry>
            ([this](auto&& entity, auto&& visual, auto&& geometry) {
                return RemoveEntityFromRGLCb(entity, visual, geometry);
            });

    UpdateRGLEntityPoses(ecm);
}

}  // namespace rgl
