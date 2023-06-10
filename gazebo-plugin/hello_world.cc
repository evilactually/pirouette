#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  // Implement Configure callback, provided by ISystemConfigure
  // and called once at startup.
  void MyPlugin::Configure(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &/*_eventMgr*/)
  {
    // Read property from SDF
    auto linkName = _sdf->Get<std::string>("link_name");
    // Create model object, to access convenient functions
    auto model = Model(_entity);
    // Get link entity
    auto linkEntity = model->LinkByName(_ecm, linkName);
  }
  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  void MyPlugin::PostUpdate(const UpdateInfo &/*_info*/,
      const EntityComponentManager &_ecm)
  {
    // Get link pose and print it
    std::cout << worldPose(this->linkEntity, _ecm) << std::endl;
  }
  // ID of link entity
  private: Entity linkEntity;
};
// Register plugin
IGNITION_ADD_PLUGIN(MyPlugin,
                    ignition::gazebo::System,
                    MyPlugin::ISystemConfigure,
                    MyPlugin::ISystemPostUpdate)
// Add plugin alias so that we can refer to the plugin without the version
// namespace
IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"ignition::gazebo::systems::MyPlugin")