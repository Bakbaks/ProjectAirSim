"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.
Pytest end-end test script for hello_drone.py functionality
"""

import asyncio
import os
import pytest
import time
import numpy as np

from projectairsim import Drone, ProjectAirSimClient, World
from projectairsim.types import ImageType
from projectairsim.utils import projectairsim_log, unpack_image

from image_validation_utils import assert_rgb_scene_image_valid

# ---------------------------------------------------------------------------
# Optional reference image (PNG/JPG) for similarity regression on DownCamera RGB.
# Leave empty to skip reference correlation; set a path (or env override below)
# after capturing a baseline from this test in your Blocks environment.
# ---------------------------------------------------------------------------
HELLO_DRONE_REFERENCE_IMAGE_PATH: str = ""
HELLO_DRONE_IMAGE_SIMILARITY_MIN: float = 0.85

# Env overrides (optional): PROJECTAIRSIM_TEST_HELLO_DRONE_REF_IMAGE,
# PROJECTAIRSIM_TEST_HELLO_DRONE_SIM_MIN
HELLO_DRONE_REF_IMAGE = (
    os.environ.get("PROJECTAIRSIM_TEST_HELLO_DRONE_REF_IMAGE", "").strip()
    or HELLO_DRONE_REFERENCE_IMAGE_PATH.strip()
)
HELLO_DRONE_SIM_MIN = float(
    os.environ.get(
        "PROJECTAIRSIM_TEST_HELLO_DRONE_SIM_MIN", str(HELLO_DRONE_IMAGE_SIMILARITY_MIN)
    )
)


def _assert_rgb_message(img_msg, context: str) -> None:
    assert_rgb_scene_image_valid(
        img_msg,
        reference_image_path="",
        min_similarity_to_reference=HELLO_DRONE_SIM_MIN,
        min_gray_std=2.0,
    )


def _assert_depth_message(img_msg, context: str) -> None:
    assert img_msg is not None and "data" in img_msg
    arr = unpack_image(img_msg)
    assert arr.size > 0, f"{context}: empty depth buffer"
    assert float(np.std(arr)) > 0.05, f"{context}: flat depth"


def check_image_rgb(img_msg):
    _assert_rgb_message(img_msg, "rgb_stream")


def check_image_depth(img_msg):
    _assert_depth_message(img_msg, "depth_stream")


def check_imu(imu_msg):
    assert len(imu_msg) > 0
    orientation = imu_msg["orientation"]
    lin_accel = imu_msg["linear_acceleration"]
    ang_vel = imu_msg["angular_velocity"]
    assert -1.0 <= orientation["w"] <= 1.0
    assert -1.0 <= orientation["x"] <= 1.0
    assert -1.0 <= orientation["y"] <= 1.0
    assert -1.0 <= orientation["z"] <= 1.0
    assert -20.0 <= lin_accel["x"] <= 20.0
    assert -20.0 <= lin_accel["y"] <= 20.0
    assert -20.0 <= lin_accel["z"] <= 20.0
    assert -5.0 <= ang_vel["x"] <= 5.0
    assert -5.0 <= ang_vel["y"] <= 5.0
    assert -5.0 <= ang_vel["z"] <= 5.0


async def wait_for_pose_change(multirotor, prev_pose, timeout=2.0):
    start = time.time()
    while True:
        pose = multirotor.robot_actual_pose
        if pose is not None and pose != prev_pose:
            return pose
        if time.time() - start > timeout:
            pytest.fail("Timeout waiting for pose update")
        await asyncio.sleep(0.05)


@pytest.fixture(scope="class")
def multirotor():
    class ProjectAirSimTestObject:
        client = ProjectAirSimClient()
        client.connect()
        world = World(client, "scene_test_drone.jsonc", 1)
        drone = Drone(client, world, "Drone1")
        robot_actual_pose = None

        def robot_actual_pose_callback(self, topic, message):
            self.robot_actual_pose = message

    multirotor_obj = ProjectAirSimTestObject()
    yield multirotor_obj

    print("\nTeardown client...")
    multirotor_obj.client.disconnect()


class TestClientBase:
    async def main(self, multirotor):
        print("start")
        drone = multirotor.drone
        client = multirotor.client
        world = multirotor.world

        client.subscribe(
            drone.robot_info["actual_pose"], multirotor.robot_actual_pose_callback
        )

        timeout = time.time() + 5
        multirotor.robot_actual_pose = None
        while multirotor.robot_actual_pose is None:
            if time.time() > timeout:
                pytest.fail("Timeout waiting for a pose message update")
            await asyncio.sleep(0.1)

        client.subscribe(
            drone.sensors["DownCamera"]["scene_camera"],
            lambda _, rgb: check_image_rgb(rgb),
        )
        client.subscribe(
            drone.sensors["DownCamera"]["depth_camera"],
            lambda _, depth: check_image_depth(depth),
        )
        client.subscribe(
            drone.sensors["IMU1"]["imu_kinematics"],
            lambda _, imu: check_imu(imu),
        )

        world.pause()
        snap = drone.get_images("DownCamera", [ImageType.SCENE])[ImageType.SCENE]
        assert_rgb_scene_image_valid(
            snap,
            reference_image_path=HELLO_DRONE_REF_IMAGE,
            min_similarity_to_reference=HELLO_DRONE_SIM_MIN,
            min_gray_std=2.0,
        )
        world.resume()

        drone.enable_api_control()
        drone.arm()

        prev_pose = multirotor.robot_actual_pose
        move_up = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=-2.0, duration=2.0
        )
        projectairsim_log().info("Move-Up invoked")
        await move_up
        projectairsim_log().info("Move-Up completed")
        new_pose = await wait_for_pose_change(multirotor, prev_pose)
        assert new_pose["position"]["z"] < prev_pose["position"]["z"]

        prev_pose = new_pose
        move_north = await drone.move_by_velocity_async(
            v_north=2.0, v_east=0.0, v_down=0.0, duration=2.0
        )
        projectairsim_log().info("Move-North invoked")
        await move_north
        projectairsim_log().info("Move-North completed")
        new_pose = await wait_for_pose_change(multirotor, prev_pose)
        assert new_pose["position"]["x"] > prev_pose["position"]["x"]

        prev_pose = new_pose
        move_west = await drone.move_by_velocity_async(
            v_north=0.0, v_east=-2.0, v_down=0.0, duration=2.0
        )
        projectairsim_log().info("Move-West invoked")
        await move_west
        projectairsim_log().info("Move-West completed")
        new_pose = await wait_for_pose_change(multirotor, prev_pose)
        assert new_pose["position"]["y"] < prev_pose["position"]["y"]

        prev_pose = new_pose
        move_south = await drone.move_by_velocity_async(
            v_north=-2.0, v_east=0.0, v_down=0.0, duration=2.0
        )
        projectairsim_log().info("Move-South invoked")
        await move_south
        projectairsim_log().info("Move-South completed")
        new_pose = await wait_for_pose_change(multirotor, prev_pose)
        assert new_pose["position"]["x"] < prev_pose["position"]["x"]

        prev_pose = new_pose
        move_east = await drone.move_by_velocity_async(
            v_north=0.0, v_east=2.0, v_down=0.0, duration=2.0
        )
        projectairsim_log().info("Move-East invoked")
        await move_east
        projectairsim_log().info("Move-East completed")
        new_pose = await wait_for_pose_change(multirotor, prev_pose)
        assert new_pose["position"]["y"] > prev_pose["position"]["y"]

        prev_pose = new_pose
        move_down = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=2.0, duration=4.0
        )
        projectairsim_log().info("Move-Down invoked")
        await move_down
        projectairsim_log().info("Move-Down completed")
        new_pose = await wait_for_pose_change(multirotor, prev_pose)
        assert new_pose["position"]["z"] > prev_pose["position"]["z"]

        drone.disarm()
        drone.disable_api_control()

        client.disconnect()

    def test_hello_drone(self, multirotor):
        asyncio.run(self.main(multirotor))
