"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2026 IAMAI CONSULTING CORP
MIT License.

Loads a JSBSim Cessna 172 configured for PX4 SITL and monitors the aircraft.

Run PX4 SITL with the Generic Standard Plane airframe before starting this
script. The script loads the scene, opens the configured cameras, and prints
basic telemetry while PX4 or QGroundControl drives the aircraft.
"""

import argparse
import asyncio
from pathlib import Path

from projectairsim import Drone, ProjectAirSimClient, World
from projectairsim.image_utils import ImageDisplay
from projectairsim.utils import projectairsim_log


async def main(scene_file: str, telemetry_period: float) -> None:
    client = ProjectAirSimClient()
    image_display = ImageDisplay()

    try:
        client.connect()

        world = World(client, scene_file, delay_after_load_sec=2)
        drone = Drone(client, world, "c172_px4")

        chase_cam_window = "ChaseCam"
        image_display.add_chase_cam(chase_cam_window)
        client.subscribe(
            drone.sensors["Chase"]["scene_camera"],
            lambda _, image: image_display.receive(image, chase_cam_window),
        )

        rgb_name = "Down-RGB"
        image_display.add_image(rgb_name, subwin_idx=0)
        client.subscribe(
            drone.sensors["DownCamera"]["scene_camera"],
            lambda _, image: image_display.receive(image, rgb_name),
        )

        depth_name = "Down-Depth"
        image_display.add_image(depth_name, subwin_idx=2)
        client.subscribe(
            drone.sensors["DownCamera"]["depth_camera"],
            lambda _, image: image_display.receive(image, depth_name),
        )

        image_display.start()

        projectairsim_log().info("Scene loaded. PX4 should now own the flight controls.")
        projectairsim_log().info(
            "Use PX4 Generic Standard Plane and arm from PX4/QGC."
        )

        while True:
            kinematics = drone.get_ground_truth_kinematics()
            position = kinematics["pose"]["position"]
            linear_velocity = kinematics["twist"]["linear"]
            angular_velocity = kinematics["twist"]["angular"]

            projectairsim_log().info(
                "NED pos=(%.2f, %.2f, %.2f) m | vel=(%.2f, %.2f, %.2f) m/s | rates=(%.2f, %.2f, %.2f) rad/s"
                % (
                    position["x"],
                    position["y"],
                    position["z"],
                    linear_velocity["x"],
                    linear_velocity["y"],
                    linear_velocity["z"],
                    angular_velocity["x"],
                    angular_velocity["y"],
                    angular_velocity["z"],
                )
            )
            await asyncio.sleep(telemetry_period)

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        client.disconnect()
        image_display.stop()


if __name__ == "__main__":
    default_scene = (
        Path(__file__).resolve().parent / "sim_config" / "scene_cessna172_px4_sitl.jsonc"
    )

    parser = argparse.ArgumentParser(description="Load and monitor the PX4 C172 scene.")
    parser.add_argument(
        "--scene",
        default=str(default_scene),
        help="Path to the scene jsonc file to load.",
    )
    parser.add_argument(
        "--telemetry-period",
        type=float,
        default=2.0,
        help="Seconds between telemetry log lines.",
    )
    args = parser.parse_args()

    try:
        asyncio.run(main(args.scene, args.telemetry_period))
    except KeyboardInterrupt:
        projectairsim_log().info("Stopped by user.")
