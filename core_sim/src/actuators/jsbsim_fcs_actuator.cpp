// Copyright (C) Microsoft Corporation.
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#include "core_sim/actuators/jsbsim_fcs_actuator.hpp"

#include <memory>
#include <string>

#include "actuator_impl.hpp"
#include "constant.hpp"
#include "core_sim/actuators/actuator.hpp"
#include "core_sim/actuators/control_mapper.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/physics_common_types.hpp"
#include "json.hpp"

// JSBSim include (requires RTTI, must be in .cpp not .hpp)
#include "FGFDMExec.h"

namespace microsoft {
namespace projectairsim {

using json = nlohmann::json;

//------------------------------------------------------------------------------
// Helper: Map JSBSimFCSType to ActuatorType enum

static ActuatorType FCSTypeToActuatorType(JSBSimFCSType fcs_type) {
  switch (fcs_type) {
    case JSBSimFCSType::kThrottle:
      return ActuatorType::kJSBSimThrottle;
    case JSBSimFCSType::kAileron:
      return ActuatorType::kJSBSimAileron;
    case JSBSimFCSType::kElevator:
      return ActuatorType::kJSBSimElevator;
    case JSBSimFCSType::kRudder:
      return ActuatorType::kJSBSimRudder;
    case JSBSimFCSType::kFlap:
      return ActuatorType::kJSBSimFlap;
    case JSBSimFCSType::kLandingGear:
      return ActuatorType::kJSBSimLandingGear;
    default:
      return ActuatorType::kJSBSimThrottle;
  }
}

/// Returns the default JSBSim property name for the given FCS type.
/// For throttle types, engine_index is appended as an array subscript.
static std::string GetDefaultJSBSimProperty(JSBSimFCSType fcs_type,
                                            int engine_index) {
  switch (fcs_type) {
    case JSBSimFCSType::kThrottle:
      return "fcs/throttle-cmd-norm[" + std::to_string(engine_index >= 0 ? engine_index : 0) + "]";
    case JSBSimFCSType::kAileron:
      return "fcs/aileron-cmd-norm";
    case JSBSimFCSType::kElevator:
      return "fcs/elevator-cmd-norm";
    case JSBSimFCSType::kRudder:
      return "fcs/rudder-cmd-norm";
    case JSBSimFCSType::kFlap:
      return "fcs/flap-cmd-norm";
    case JSBSimFCSType::kLandingGear:
      return "gear/gear-cmd-norm";
    default:
      return "fcs/throttle-cmd-norm[0]";
  }
}

/// Returns the default range for the FCS type.
static void GetDefaultRange(JSBSimFCSType fcs_type, float& range_min,
                            float& range_max) {
  switch (fcs_type) {
    case JSBSimFCSType::kAileron:
    case JSBSimFCSType::kElevator:
    case JSBSimFCSType::kRudder:
      range_min = -1.0f;
      range_max = 1.0f;
      break;
    case JSBSimFCSType::kThrottle:
    case JSBSimFCSType::kFlap:
    case JSBSimFCSType::kLandingGear:
    default:
      range_min = 0.0f;
      range_max = 1.0f;
      break;
  }
}

/// Returns the component name for a given FCS type.
static const char* GetComponentName(JSBSimFCSType fcs_type) {
  switch (fcs_type) {
    case JSBSimFCSType::kThrottle:
      return Constant::Component::jsbsim_throttle;
    case JSBSimFCSType::kAileron:
      return Constant::Component::jsbsim_aileron;
    case JSBSimFCSType::kElevator:
      return Constant::Component::jsbsim_elevator;
    case JSBSimFCSType::kRudder:
      return Constant::Component::jsbsim_rudder;
    case JSBSimFCSType::kFlap:
      return Constant::Component::jsbsim_flap;
    case JSBSimFCSType::kLandingGear:
      return Constant::Component::jsbsim_landing_gear;
    default:
      return Constant::Component::jsbsim_throttle;
  }
}

//------------------------------------------------------------------------------
// Forward declarations

class JSBSimFCSActuator::Loader {
 public:
  explicit Loader(JSBSimFCSActuator::Impl& impl);

  void Load(const json& json);

 private:
  void LoadFCSSettings(const json& json);

  JSBSimFCSActuator::Impl& impl_;
};

class JSBSimFCSActuator::Impl : public ActuatorImpl {
 public:
  Impl(JSBSimFCSType fcs_type, const std::string& id, bool is_enabled,
       const std::string& parent_link, const std::string& child_link,
       const Logger& logger, const TopicManager& topic_manager,
       const std::string& parent_topic_path,
       const ServiceManager& service_manager,
       const StateManager& state_manager);

  void Load(ConfigJson config_json);

  void CreateTopics(void);

  void OnBeginUpdate(void) override;

  void OnEndUpdate(void) override;

  JSBSimFCSType GetFCSType() const;

  const JSBSimFCSSettings& GetFCSSettings() const;

  void SetJSBSimModel(std::shared_ptr<JSBSim::FGFDMExec> model);

  void UpdateActuatorOutput(std::vector<float>&& control_signals,
                            const TimeNano sim_dt_nanos);

 private:
  friend class JSBSimFCSActuator::Loader;

  JSBSimFCSType fcs_type_;
  JSBSimFCSSettings fcs_settings_;
  ControlMapper control_mapper_;
  std::shared_ptr<JSBSim::FGFDMExec> jsbsim_model_;
  std::vector<Topic> topics_;

  JSBSimFCSActuator::Loader loader_;
};

//------------------------------------------------------------------------------
// class JSBSimFCSActuator

JSBSimFCSActuator::JSBSimFCSActuator(void)
    : Actuator(std::shared_ptr<ActuatorImpl>(nullptr)) {}

JSBSimFCSActuator::JSBSimFCSActuator(
    JSBSimFCSType fcs_type, const std::string& id, bool is_enabled,
    const std::string& parent_link, const std::string& child_link,
    const Logger& logger, const TopicManager& topic_manager,
    const std::string& parent_topic_path,
    const ServiceManager& service_manager,
    const StateManager& state_manager)
    : Actuator(std::shared_ptr<ActuatorImpl>(new JSBSimFCSActuator::Impl(
          fcs_type, id, is_enabled, parent_link, child_link, logger,
          topic_manager, parent_topic_path, service_manager, state_manager))) {}

void JSBSimFCSActuator::Load(ConfigJson config_json) {
  static_cast<JSBSimFCSActuator::Impl*>(pimpl_.get())->Load(config_json);
}

JSBSimFCSType JSBSimFCSActuator::GetFCSType() const {
  return static_cast<JSBSimFCSActuator::Impl*>(pimpl_.get())->GetFCSType();
}

const JSBSimFCSSettings& JSBSimFCSActuator::GetFCSSettings() const {
  return static_cast<JSBSimFCSActuator::Impl*>(pimpl_.get())->GetFCSSettings();
}

void JSBSimFCSActuator::SetJSBSimModel(
    std::shared_ptr<JSBSim::FGFDMExec> model) {
  static_cast<JSBSimFCSActuator::Impl*>(pimpl_.get())->SetJSBSimModel(model);
}

void JSBSimFCSActuator::UpdateActuatorOutput(
    std::vector<float>&& control_signals, const TimeNano sim_dt_nanos) {
  static_cast<JSBSimFCSActuator::Impl*>(pimpl_.get())
      ->UpdateActuatorOutput(std::move(control_signals), sim_dt_nanos);
}

//------------------------------------------------------------------------------
// class JSBSimFCSActuator::Impl

JSBSimFCSActuator::Impl::Impl(
    JSBSimFCSType fcs_type, const std::string& id, bool is_enabled,
    const std::string& parent_link, const std::string& child_link,
    const Logger& logger, const TopicManager& topic_manager,
    const std::string& parent_topic_path,
    const ServiceManager& service_manager,
    const StateManager& state_manager)
    : ActuatorImpl(FCSTypeToActuatorType(fcs_type), id, is_enabled, parent_link,
                   child_link, GetComponentName(fcs_type), logger,
                   topic_manager, parent_topic_path, service_manager,
                   state_manager),
      fcs_type_(fcs_type),
      fcs_settings_(),
      control_mapper_(),
      jsbsim_model_(nullptr),
      topics_(),
      loader_(*this) {
  // Set default range based on FCS type
  GetDefaultRange(fcs_type_, fcs_settings_.range_min, fcs_settings_.range_max);

  SetTopicPath();
  CreateTopics();
}

void JSBSimFCSActuator::Impl::Load(ConfigJson config_json) {
  json json = config_json;
  loader_.Load(json);
}

void JSBSimFCSActuator::Impl::CreateTopics() {}

void JSBSimFCSActuator::Impl::OnBeginUpdate() {}

void JSBSimFCSActuator::Impl::OnEndUpdate() {}

JSBSimFCSType JSBSimFCSActuator::Impl::GetFCSType() const {
  return fcs_type_;
}

const JSBSimFCSSettings& JSBSimFCSActuator::Impl::GetFCSSettings() const {
  return fcs_settings_;
}

void JSBSimFCSActuator::Impl::SetJSBSimModel(
    std::shared_ptr<JSBSim::FGFDMExec> model) {
  jsbsim_model_ = model;
}

void JSBSimFCSActuator::Impl::UpdateActuatorOutput(
    std::vector<float>&& control_signals, const TimeNano sim_dt_nanos) {
  if (!enabled_ || is_fault_injected_) return;

  if (jsbsim_model_ == nullptr) {
    logger_.LogWarning(
        name_,
        "[%s] JSBSim model not set for FCS actuator, skipping property write.",
        id_.c_str());
    return;
  }

  if (control_signals.empty()) {
    return;
  }

  // Map the control signal through the control mapper (handles
  // domain/range/scale/clamping)
  float mapped_value = control_mapper_(control_signals[0]);

  // Clamp to the FCS property's valid range
  mapped_value =
      std::max(fcs_settings_.range_min,
               std::min(fcs_settings_.range_max, mapped_value));

  // Write to JSBSim property
  jsbsim_model_->SetPropertyValue(fcs_settings_.jsbsim_property, mapped_value);
}

//------------------------------------------------------------------------------
// class JSBSimFCSActuator::Loader

JSBSimFCSActuator::Loader::Loader(JSBSimFCSActuator::Impl& impl)
    : impl_(impl) {}

void JSBSimFCSActuator::Loader::Load(const json& json) {
  impl_.logger_.LogVerbose(
      impl_.name_, "[%s] Loading 'jsbsim-fcs' actuator.", impl_.id_.c_str());

  LoadFCSSettings(json);

  impl_.is_loaded_ = true;

  impl_.logger_.LogVerbose(
      impl_.name_, "[%s] 'jsbsim-fcs' actuator loaded.", impl_.id_.c_str());
}

void JSBSimFCSActuator::Loader::LoadFCSSettings(const json& json) {
  impl_.logger_.LogVerbose(impl_.name_, "Loading 'jsbsim-fcs-settings'.");

  // Check if optional jsbsim-fcs-settings block exists
  auto has_settings =
      json.find(Constant::Config::jsbsim_fcs_settings) != json.end();

  if (has_settings) {
    auto settings_json =
        JsonUtils::GetJsonObject(json, Constant::Config::jsbsim_fcs_settings);

    // Override engine index (for throttle actuators)
    impl_.fcs_settings_.engine_index = JsonUtils::GetNumber<int>(
        settings_json, Constant::Config::engine_index,
        impl_.fcs_settings_.engine_index);

    // Override JSBSim property name
    std::string custom_property = JsonUtils::GetString(
        settings_json, Constant::Config::jsbsim_property, "");
    if (!custom_property.empty()) {
      impl_.fcs_settings_.jsbsim_property = custom_property;
    }

    // Override range
    impl_.fcs_settings_.range_min = JsonUtils::GetNumber<float>(
        settings_json, Constant::Config::input_min,
        impl_.fcs_settings_.range_min);
    impl_.fcs_settings_.range_max = JsonUtils::GetNumber<float>(
        settings_json, Constant::Config::input_max,
        impl_.fcs_settings_.range_max);

    // Load control input map if present
    if (settings_json.find(Constant::Config::input_map) !=
        settings_json.end()) {
      impl_.control_mapper_.Load(JsonUtils::GetJsonObject(
          settings_json, Constant::Config::input_map));
    }
  }

  // If JSBSim property was not explicitly set, use the default based on type
  if (impl_.fcs_settings_.jsbsim_property.empty()) {
    impl_.fcs_settings_.jsbsim_property = GetDefaultJSBSimProperty(
        impl_.fcs_type_, impl_.fcs_settings_.engine_index);
  }

  impl_.logger_.LogVerbose(
      impl_.name_, "[%s] jsbsim-fcs property: '%s', range: [%.2f, %.2f]",
      impl_.id_.c_str(), impl_.fcs_settings_.jsbsim_property.c_str(),
      impl_.fcs_settings_.range_min, impl_.fcs_settings_.range_max);

  impl_.logger_.LogVerbose(impl_.name_, "'jsbsim-fcs-settings' loaded.");
}

}  // namespace projectairsim
}  // namespace microsoft
