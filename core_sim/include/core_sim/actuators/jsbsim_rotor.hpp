// Copyright (C) Microsoft Corporation.
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#ifndef CORE_SIM_INCLUDE_CORE_SIM_ACTUATORS_JSBSIM_ROTOR_HPP_
#define CORE_SIM_INCLUDE_CORE_SIM_ACTUATORS_JSBSIM_ROTOR_HPP_

#include <memory>
#include <string>

#include "core_sim/actuators/actuator.hpp"
#include "core_sim/physics_common_types.hpp"
#include "core_sim/transforms/transform.hpp"

// Forward declaration to avoid including JSBSim headers with RTTI requirements
// in Unreal Engine plugin code (UE is compiled without RTTI)
namespace JSBSim {
class FGFDMExec;
}

namespace microsoft {
namespace projectairsim {

class ConfigJson;
class Logger;
class ActuatorImpl;
class TopicManager;
class ServiceManager;
class StateManager;

// In NED system, +ve torque would generate clockwise rotation
enum class JSBSimRotorTurningDirection : int {
  kCCW = -1,
  kCW = 1
};

/// Settings loaded from the robot JSON config for a JSBSim rotor actuator.
struct JSBSimRotorSettings {
  /// The JSBSim property path to write the control signal to.
  std::string jsbsim_cmd;

  /// The JSBSim property path to read the actuator state from.
  /// Defaults to jsbsim_cmd if absent.
  std::string jsbsim_state;

  /// Turning direction for visual mesh rotation.
  JSBSimRotorTurningDirection turning_direction =
      JSBSimRotorTurningDirection::kCW;

  /// Normal vector for the rotation axis (NED, default is -Z = up thrust).
  Vector3 normal_vector = Vector3(0, 0, -1);
};

//------------------------------------------------------------------------------
// class JSBSimRotor
//
// A bridge actuator that receives control signals from a controller
// (e.g. SimpleFlight, PX4) and writes them as properties on the JSBSim
// flight dynamics model.  Unlike the Rotor actuator which computes
// thrust/torque from physics coefficients, this actuator delegates all
// physics to JSBSim and only handles the visual mesh spinning.

class JSBSimRotor : public Actuator {
 public:
  JSBSimRotor();

  /// Returns the loaded settings.
  const JSBSimRotorSettings& GetSettings() const;

  /// Injects the JSBSim model reference. Must be called after the model is
  /// initialized (by Robot::Impl after InitializeJSBSimModel()).
  void SetJSBSimModel(std::shared_ptr<JSBSim::FGFDMExec> model);

  /// Reads the current state from the JSBSim property (jsbsim-state).
  /// Returns -1.0f if JSBSim model is not set or property is empty.
  float GetJSBSimState() const;

  /// Returns the current visual rotation angle of the rotor mesh (radians).
  float GetAngle() const;

  /// Returns the current visual rotating speed (rad/s, estimated from signal).
  float GetRotatingSpeed() const;

  /// Returns the actuated transforms for child link mesh spinning.
  const ActuatedTransforms& GetActuatedTransforms() const;

  /// Receives control signal from controller, writes to JSBSim, updates visual.
  void UpdateActuatorOutput(std::vector<float>&& control_signals,
                            const TimeNano sim_dt_nanos) override;

 private:
  friend class Robot;
  friend class ActuatorImpl;

  JSBSimRotor(const std::string& id, bool is_enabled,
              const std::string& parent_link, const std::string& child_link,
              const Logger& logger, const TopicManager& topic_manager,
              const std::string& parent_topic_path,
              const ServiceManager& service_manager,
              const StateManager& state_manager);

  void Load(ConfigJson config_json) override;

  class Impl;
  class Loader;
};

}  // namespace projectairsim
}  // namespace microsoft

#endif  // CORE_SIM_INCLUDE_CORE_SIM_ACTUATORS_JSBSIM_ROTOR_HPP_
