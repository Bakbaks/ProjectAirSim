// Copyright (C) Microsoft Corporation.
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#include "core_sim/actor/robot.hpp"
#include "core_sim/actuators/actuator.hpp"
#include "core_sim/actuators/jsbsim_fcs_actuator.hpp"
#include "core_sim/config_json.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/service_manager.hpp"
#include "gtest/gtest.h"
#include "json.hpp"
#include "state_manager.hpp"
#include "topic_manager.hpp"

namespace microsoft {
namespace projectairsim {

class JSBSimFCSScene {
 public:
  static Robot MakeRobot(const std::string& id) {
    auto logger_callback = [](const std::string& component, LogLevel level,
                              const std::string& message) {};
    Logger logger(logger_callback);
    Transform origin = {{0, 0, 0}, {1, 0, 0, 0}};
    return Robot("TestRobotID", origin, logger, TopicManager(logger), "",
                 ServiceManager(logger), StateManager(logger), "");
  }

  static void LoadRobot(Robot& robot, ConfigJson config_json) {
    robot.Load(config_json);
  }
};

}  // namespace projectairsim
}  // namespace microsoft

namespace projectairsim = microsoft::projectairsim;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Test: Load a single jsbsim-throttle actuator

TEST(JSBSimFCSActuator, LoadsThrottleActuator) {
  json config_json = R"({
    "links": [ { "name": "Frame" } ],
    "actuators": [
      {
        "name": "throttle_0",
        "type": "jsbsim-throttle",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      }
    ]
  })"_json;

  auto robot = projectairsim::JSBSimFCSScene::MakeRobot("TestRobot");
  projectairsim::JSBSimFCSScene::LoadRobot(robot, config_json);
  auto& actuators = robot.GetActuators();
  ASSERT_EQ(actuators.size(), 1);
  EXPECT_EQ(actuators[0]->GetType(),
            projectairsim::ActuatorType::kJSBSimThrottle);
}

// ---------------------------------------------------------------------------
// Test: Load all 6 FCS actuator types at once

TEST(JSBSimFCSActuator, LoadsAllFCSTypes) {
  json config_json = R"({
    "links": [ { "name": "Frame" } ],
    "actuators": [
      {
        "name": "throttle_0",
        "type": "jsbsim-throttle",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      },
      {
        "name": "aileron",
        "type": "jsbsim-aileron",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      },
      {
        "name": "elevator",
        "type": "jsbsim-elevator",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      },
      {
        "name": "rudder",
        "type": "jsbsim-rudder",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      },
      {
        "name": "flap",
        "type": "jsbsim-flap",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      },
      {
        "name": "landing_gear",
        "type": "jsbsim-landing-gear",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      }
    ]
  })"_json;

  auto robot = projectairsim::JSBSimFCSScene::MakeRobot("TestRobot");
  projectairsim::JSBSimFCSScene::LoadRobot(robot, config_json);
  auto& actuators = robot.GetActuators();
  ASSERT_EQ(actuators.size(), 6);

  EXPECT_EQ(actuators[0]->GetType(),
            projectairsim::ActuatorType::kJSBSimThrottle);
  EXPECT_EQ(actuators[1]->GetType(),
            projectairsim::ActuatorType::kJSBSimAileron);
  EXPECT_EQ(actuators[2]->GetType(),
            projectairsim::ActuatorType::kJSBSimElevator);
  EXPECT_EQ(actuators[3]->GetType(),
            projectairsim::ActuatorType::kJSBSimRudder);
  EXPECT_EQ(actuators[4]->GetType(),
            projectairsim::ActuatorType::kJSBSimFlap);
  EXPECT_EQ(actuators[5]->GetType(),
            projectairsim::ActuatorType::kJSBSimLandingGear);
}

// ---------------------------------------------------------------------------
// Test: Throttle with custom engine-index loads custom JSBSim property

TEST(JSBSimFCSActuator, ThrottleWithEngineIndex) {
  json config_json = R"({
    "links": [ { "name": "Frame" } ],
    "actuators": [
      {
        "name": "throttle_1",
        "type": "jsbsim-throttle",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame",
        "jsbsim-fcs-settings": {
          "engine-index": 1
        }
      }
    ]
  })"_json;

  auto robot = projectairsim::JSBSimFCSScene::MakeRobot("TestRobot");
  projectairsim::JSBSimFCSScene::LoadRobot(robot, config_json);
  auto& actuators = robot.GetActuators();
  ASSERT_EQ(actuators.size(), 1);

  auto* fcs = static_cast<projectairsim::JSBSimFCSActuator*>(actuators[0].get());
  EXPECT_EQ(fcs->GetFCSSettings().jsbsim_property, "fcs/throttle-cmd-norm[1]");
  EXPECT_EQ(fcs->GetFCSSettings().engine_index, 1);
}

// ---------------------------------------------------------------------------
// Test: Custom JSBSim property override

TEST(JSBSimFCSActuator, CustomPropertyOverride) {
  json config_json = R"({
    "links": [ { "name": "Frame" } ],
    "actuators": [
      {
        "name": "custom_aileron",
        "type": "jsbsim-aileron",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame",
        "jsbsim-fcs-settings": {
          "jsbsim-property": "fcs/left-aileron-cmd-norm"
        }
      }
    ]
  })"_json;

  auto robot = projectairsim::JSBSimFCSScene::MakeRobot("TestRobot");
  projectairsim::JSBSimFCSScene::LoadRobot(robot, config_json);
  auto& actuators = robot.GetActuators();
  ASSERT_EQ(actuators.size(), 1);

  auto* fcs = static_cast<projectairsim::JSBSimFCSActuator*>(actuators[0].get());
  EXPECT_EQ(fcs->GetFCSSettings().jsbsim_property, "fcs/left-aileron-cmd-norm");
}

// ---------------------------------------------------------------------------
// Test: Default range values for different FCS types

TEST(JSBSimFCSActuator, DefaultRangeValues) {
  json config_json = R"({
    "links": [ { "name": "Frame" } ],
    "actuators": [
      {
        "name": "throttle_0",
        "type": "jsbsim-throttle",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      },
      {
        "name": "aileron",
        "type": "jsbsim-aileron",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame"
      }
    ]
  })"_json;

  auto robot = projectairsim::JSBSimFCSScene::MakeRobot("TestRobot");
  projectairsim::JSBSimFCSScene::LoadRobot(robot, config_json);
  auto& actuators = robot.GetActuators();
  ASSERT_EQ(actuators.size(), 2);

  // Throttle: [0, 1]
  auto* throttle =
      static_cast<projectairsim::JSBSimFCSActuator*>(actuators[0].get());
  EXPECT_FLOAT_EQ(throttle->GetFCSSettings().range_min, 0.0f);
  EXPECT_FLOAT_EQ(throttle->GetFCSSettings().range_max, 1.0f);

  // Aileron: [-1, 1]
  auto* aileron =
      static_cast<projectairsim::JSBSimFCSActuator*>(actuators[1].get());
  EXPECT_FLOAT_EQ(aileron->GetFCSSettings().range_min, -1.0f);
  EXPECT_FLOAT_EQ(aileron->GetFCSSettings().range_max, 1.0f);
}

// ---------------------------------------------------------------------------
// Test: Existing rotor actuators still load correctly (backward compat)

TEST(JSBSimFCSActuator, BackwardCompatRotorStillLoads) {
  json config_json = R"({
    "links": [ { "name": "Frame" } ],
    "actuators": [
      {
        "name": "fcs/throttle-cmd-norm",
        "type": "rotor",
        "enabled": true,
        "parent-link": "Frame",
        "child-link": "Frame",
        "origin": {
          "xyz": "0 0 0",
          "rpy-deg": "0 0 0"
        }
      }
    ]
  })"_json;

  auto robot = projectairsim::JSBSimFCSScene::MakeRobot("TestRobot");
  projectairsim::JSBSimFCSScene::LoadRobot(robot, config_json);
  auto& actuators = robot.GetActuators();
  ASSERT_EQ(actuators.size(), 1);
  EXPECT_EQ(actuators[0]->GetType(), projectairsim::ActuatorType::kRotor);
}
