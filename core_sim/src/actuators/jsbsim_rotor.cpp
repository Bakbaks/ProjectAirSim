// Copyright (C) Microsoft Corporation.
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#include "core_sim/actuators/jsbsim_rotor.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "actuator_impl.hpp"
#include "constant.hpp"
#include "core_sim/actuators/actuator.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/physics_common_types.hpp"
#include "json.hpp"

// JSBSim include (requires RTTI, must be in .cpp not .hpp)
#include "FGFDMExec.h"

namespace microsoft {
namespace projectairsim {

using json = nlohmann::json;

//------------------------------------------------------------------------------
// Forward declarations

class JSBSimRotor::Loader {
 public:
  explicit Loader(JSBSimRotor::Impl& impl);

  void Load(const json& json);

 private:
  void LoadSettings(const json& json);

  JSBSimRotor::Impl& impl_;
};

class JSBSimRotor::Impl : public ActuatorImpl {
 public:
  Impl(const std::string& id, bool is_enabled, const std::string& parent_link,
       const std::string& child_link, const Logger& logger,
       const TopicManager& topic_manager, const std::string& parent_topic_path,
       const ServiceManager& service_manager,
       const StateManager& state_manager);

  void Load(ConfigJson config_json);

  void CreateTopics();

  void OnBeginUpdate() override;

  void OnEndUpdate() override;

  const JSBSimRotorSettings& GetSettings() const;

  void SetJSBSimModel(std::shared_ptr<JSBSim::FGFDMExec> model);

  float GetJSBSimState() const;

  float GetAngle() const;

  float GetRotatingSpeed() const;

  const ActuatedTransforms& GetActuatedTransforms() const;

  void UpdateActuatorOutput(std::vector<float>&& control_signals,
                            const TimeNano sim_dt_nanos);

 private:
  friend class JSBSimRotor::Loader;

  JSBSimRotor::Loader loader_;
  JSBSimRotorSettings settings_;
  std::shared_ptr<JSBSim::FGFDMExec> jsbsim_model_;

  // Visual state for mesh spinning
  ActuatedTransforms actuated_transforms_;
  float angle_cur_;          // Absolute rotor angle (radians)
  float rotating_speed_;     // Visual rotating speed (rad/s)

  std::vector<Topic> topics_;
};

//------------------------------------------------------------------------------
// class JSBSimRotor

JSBSimRotor::JSBSimRotor()
    : Actuator(std::shared_ptr<ActuatorImpl>(nullptr)) {}

JSBSimRotor::JSBSimRotor(const std::string& id, bool is_enabled,
                         const std::string& parent_link,
                         const std::string& child_link, const Logger& logger,
                         const TopicManager& topic_manager,
                         const std::string& parent_topic_path,
                         const ServiceManager& service_manager,
                         const StateManager& state_manager)
    : Actuator(std::shared_ptr<ActuatorImpl>(new JSBSimRotor::Impl(
          id, is_enabled, parent_link, child_link, logger, topic_manager,
          parent_topic_path, service_manager, state_manager))) {}

void JSBSimRotor::Load(ConfigJson config_json) {
  static_cast<JSBSimRotor::Impl*>(pimpl_.get())->Load(config_json);
}

const JSBSimRotorSettings& JSBSimRotor::GetSettings() const {
  return static_cast<JSBSimRotor::Impl*>(pimpl_.get())->GetSettings();
}

void JSBSimRotor::SetJSBSimModel(
    std::shared_ptr<JSBSim::FGFDMExec> model) {
  static_cast<JSBSimRotor::Impl*>(pimpl_.get())->SetJSBSimModel(model);
}

float JSBSimRotor::GetJSBSimState() const {
  return static_cast<JSBSimRotor::Impl*>(pimpl_.get())->GetJSBSimState();
}

float JSBSimRotor::GetAngle() const {
  return static_cast<JSBSimRotor::Impl*>(pimpl_.get())->GetAngle();
}

float JSBSimRotor::GetRotatingSpeed() const {
  return static_cast<JSBSimRotor::Impl*>(pimpl_.get())->GetRotatingSpeed();
}

const ActuatedTransforms& JSBSimRotor::GetActuatedTransforms() const {
  return static_cast<JSBSimRotor::Impl*>(pimpl_.get())->GetActuatedTransforms();
}

void JSBSimRotor::UpdateActuatorOutput(std::vector<float>&& control_signals,
                                       const TimeNano sim_dt_nanos) {
  static_cast<JSBSimRotor::Impl*>(pimpl_.get())
      ->UpdateActuatorOutput(std::move(control_signals), sim_dt_nanos);
}

//------------------------------------------------------------------------------
// class JSBSimRotor::Impl

JSBSimRotor::Impl::Impl(const std::string& id, bool is_enabled,
                         const std::string& parent_link,
                         const std::string& child_link, const Logger& logger,
                         const TopicManager& topic_manager,
                         const std::string& parent_topic_path,
                         const ServiceManager& service_manager,
                         const StateManager& state_manager)
    : ActuatorImpl(ActuatorType::kJSBSimRotor, id, is_enabled, parent_link,
                   child_link, Constant::Component::jsbsim_rotor, logger,
                   topic_manager, parent_topic_path, service_manager,
                   state_manager),
      loader_(*this),
      settings_(),
      jsbsim_model_(nullptr),
      actuated_transforms_(),
      angle_cur_(0.0f),
      rotating_speed_(0.0f),
      topics_() {
  SetTopicPath();
  CreateTopics();
  ActuatorImpl::RegisterServiceMethods();
}

void JSBSimRotor::Impl::Load(ConfigJson config_json) {
  json json = config_json;
  loader_.Load(json);
}

void JSBSimRotor::Impl::CreateTopics() {}

void JSBSimRotor::Impl::OnBeginUpdate() {}

void JSBSimRotor::Impl::OnEndUpdate() {}

const JSBSimRotorSettings& JSBSimRotor::Impl::GetSettings() const {
  return settings_;
}

void JSBSimRotor::Impl::SetJSBSimModel(
    std::shared_ptr<JSBSim::FGFDMExec> model) {
  jsbsim_model_ = model;
}

float JSBSimRotor::Impl::GetJSBSimState() const {
  if (!settings_.jsbsim_state.empty() && jsbsim_model_ != nullptr) {
    return static_cast<float>(
        jsbsim_model_->GetPropertyValue(settings_.jsbsim_state));
  }
  return -1.0f;
}

float JSBSimRotor::Impl::GetAngle() const { return angle_cur_; }

float JSBSimRotor::Impl::GetRotatingSpeed() const { return rotating_speed_; }

const ActuatedTransforms& JSBSimRotor::Impl::GetActuatedTransforms() const {
  return actuated_transforms_;
}

void JSBSimRotor::Impl::UpdateActuatorOutput(
    std::vector<float>&& control_signals, const TimeNano sim_dt_nanos) {
  if (!enabled_ || is_fault_injected_) return;

  if (control_signals.empty()) return;

  float control_signal = std::clamp(control_signals[0], 0.0f, 1.0f);

  // Write control signal to JSBSim property
  if (jsbsim_model_ != nullptr && !settings_.jsbsim_cmd.empty()) {
    jsbsim_model_->SetPropertyValue(settings_.jsbsim_cmd,
                                    static_cast<double>(control_signal));
  }

  // Update visual mesh spinning
  // Use the control signal as a proportional visual speed indicator.
  // A nominal max visual speed of ~600 rad/s (~5730 RPM) gives a good
  // visual spinning effect.
  constexpr float kNominalMaxSpeed = 600.0f;  // rad/s
  rotating_speed_ = control_signal * kNominalMaxSpeed;

  TimeSec dt_sec = sim_dt_nanos / 1.0e9;
  int dir = static_cast<int>(settings_.turning_direction);
  float dangle = rotating_speed_ * dt_sec * dir;

  angle_cur_ += dangle;
  while (angle_cur_ < 0.0f) angle_cur_ += static_cast<float>(2.0 * M_PI);
  while (angle_cur_ > static_cast<float>(2.0 * M_PI))
    angle_cur_ -= static_cast<float>(2.0 * M_PI);

  actuated_transforms_[child_link_] = Affine3(
      AngleAxis(angle_cur_, settings_.normal_vector).toRotationMatrix());
}

//------------------------------------------------------------------------------
// class JSBSimRotor::Loader

JSBSimRotor::Loader::Loader(JSBSimRotor::Impl& impl) : impl_(impl) {}

void JSBSimRotor::Loader::Load(const json& json) {
  impl_.logger_.LogVerbose(impl_.name_,
                           "[%s] Loading 'jsbsim-rotor' actuator.",
                           impl_.id_.c_str());

  LoadSettings(json);

  impl_.is_loaded_ = true;

  impl_.logger_.LogVerbose(impl_.name_,
                           "[%s] 'jsbsim-rotor' actuator loaded.",
                           impl_.id_.c_str());
}

void JSBSimRotor::Loader::LoadSettings(const json& json) {
  impl_.logger_.LogVerbose(impl_.name_, "Loading 'jsbsim-rotor-settings'.");

  auto settings_json =
      JsonUtils::GetJsonObject(json, Constant::Config::jsbsim_rotor_settings);

  if (JsonUtils::IsEmpty(settings_json)) {
    impl_.logger_.LogWarning(
        impl_.name_,
        "[%s] 'jsbsim-rotor-settings' missing or empty. "
        "jsbsim-cmd is required for JSBSim rotor actuator.",
        impl_.id_.c_str());
    return;
  }

  // Required: JSBSim property to write control signal to
  impl_.settings_.jsbsim_cmd = JsonUtils::GetString(
      settings_json, Constant::Config::jsbsim_cmd, "");

  // Optional: JSBSim property to read state from (defaults to jsbsim-cmd)
  impl_.settings_.jsbsim_state = JsonUtils::GetString(
      settings_json, Constant::Config::jsbsim_state,
      impl_.settings_.jsbsim_cmd);

  // Optional: turning direction for visual spinning
  auto turning_dir = JsonUtils::GetString(
      settings_json, Constant::Config::turning_direction,
      Constant::Config::clock_wise);
  if (turning_dir == Constant::Config::counter_clock_wise) {
    impl_.settings_.turning_direction = JSBSimRotorTurningDirection::kCCW;
  } else {
    impl_.settings_.turning_direction = JSBSimRotorTurningDirection::kCW;
  }

  // Optional: normal vector for rotation axis
  auto nv_json =
      JsonUtils::GetJsonObject(settings_json, Constant::Config::normal_vector);
  if (!JsonUtils::IsEmpty(nv_json)) {
    impl_.settings_.normal_vector =
        JsonUtils::GetVector3(settings_json, Constant::Config::normal_vector);
  }

  impl_.logger_.LogVerbose(
      impl_.name_,
      "[%s] jsbsim-rotor: cmd='%s', state='%s', dir=%d",
      impl_.id_.c_str(), impl_.settings_.jsbsim_cmd.c_str(),
      impl_.settings_.jsbsim_state.c_str(),
      static_cast<int>(impl_.settings_.turning_direction));
}

}  // namespace projectairsim
}  // namespace microsoft
