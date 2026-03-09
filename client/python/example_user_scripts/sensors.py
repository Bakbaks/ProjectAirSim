"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates getting data from various sensor types.
"""

import asyncio
import sys
import time

from projectairsim import Drone, ProjectAirSimClient, World
from projectairsim.image_utils import ImageDisplay
from projectairsim.utils import projectairsim_log


async def demo_sensors(drone: Drone):
    # Set the drone to be ready to fly
    drone.enable_api_control()
    drone.arm()

    # Command the drone to move up in NED coordinate system for 5 seconds
    move_up_task = await drone.move_by_velocity_async(
        v_north=0.0, v_east=0.0, v_down=-1.0, duration=5.0
    )

    # Wait for the move_up_task to complete
    await move_up_task

    # Command the drone to move North in NED coordinate system for 10 seconds
    move_north_task = await drone.move_by_velocity_async(
        v_north=1.0, v_east=0.0, v_down=0.0, duration=10.0
    )

    # Wait for the move_north_task to complete
    await move_north_task

    drone.disable_api_control()
    drone.disarm()


def keep_logging_sensors() -> None:
    projectairsim_log().info("Logging sensor data continuously. Press Ctrl+C to stop.\n")
    while True:
        time.sleep(1.0)


def select_scene_file() -> str:
    scene_by_choice = {
        "1": "scene_drone_sensors.jsonc",
        "2": "scene_drone_sensors_jsbsim.jsonc",
    }

    if len(sys.argv) > 1:
        selected = sys.argv[1].strip()
        if selected in scene_by_choice:
            return scene_by_choice[selected]
        raise ValueError("Invalid parameter. Use 1 (fastphysics) or 2 (jsbsim).")

    projectairsim_log().info("Select mode: 1=fastphysics, 2=jsbsim")
    while True:
        selected = input("Enter 1 or 2: ").strip()
        if selected in scene_by_choice:
            return scene_by_choice[selected]
        print("Invalid option. Please enter 1 or 2.")


if __name__ == "__main__":
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    # Initialize an ImageDisplay object to display camera sub-windows
    image_display = ImageDisplay()

    try:
        # Connect to simulation environment
        client.connect()

        scene_file = select_scene_file()
        projectairsim_log().info(f"Using scene config: {scene_file}")

        # Create a World object to interact with the sim world and load a scene
        world = World(client, scene_file)

        # Create a Drone object to interact with a drone in the loaded sim world
        drone = Drone(client, world, "Drone1")

        # Subscribe to the Drone's sensors with a callback to receive the sensor data

        # # Subscribe to chase camera sensor
        # chase_cam_window = "ChaseCam"
        # image_display.add_chase_cam(chase_cam_window)
        # client.subscribe(
        #     drone.sensors["Chase"]["scene_camera"],
        #     lambda _, chase: image_display.receive(chase, chase_cam_window),
        # )

        # # Subscribe to the Drone's RGB Camera and display the captured image
        # rgb_name = "RGB-Image"
        # image_display.add_image(rgb_name, subwin_idx=0)
        # client.subscribe(
        #     drone.sensors["DownCamera"]["scene_camera"],
        #     lambda _, rgb: image_display.receive(rgb, rgb_name),
        # )

        # # Subscribe to the Drone's Depth Camera and display the captured image
        # depth_name = "Depth-Image"
        # image_display.add_image(depth_name, subwin_idx=1)
        # client.subscribe(
        #     drone.sensors["DownCamera"]["depth_camera"],
        #     lambda _, depth: image_display.receive(depth, depth_name),
        # )

        # # Subscribe to the Drone's Segmentation Camera and display the captured image
        # seg_name = "Seg-Image"
        # image_display.add_image(seg_name, subwin_idx=2)
        # client.subscribe(
        #     drone.sensors["DownCamera"]["segmentation_camera"],
        #     lambda _, seg: image_display.receive(seg, seg_name),
        # )

        # image_display.start()

        # Subscribe to the Drone's GPS sensor and print the GPS data
        client.subscribe(
            drone.sensors["GPS"]["gps"],
            lambda _, gps: projectairsim_log().info(f"GPS: {gps} \n"),
        )

        # Subscribe to the Drone's Barometer sensor and print the Barometer data
        client.subscribe(
            drone.sensors["Barometer"]["barometer"],
            lambda _, barometer: projectairsim_log().info(f"Barometer: {barometer} \n"),
        )

        # Subscribe to the Drone's Distance sensor and print the Distance data
        client.subscribe(
            drone.sensors["DistanceSensor"]["distance_sensor"],
            lambda _, distance: projectairsim_log().info(f"DistanceSensor: {distance} \n"),
        )

        # Subscribe to the Drone's IMU sensor and print the IMU data
        client.subscribe(
            drone.sensors["IMU1"]["imu_kinematics"],
            lambda _, imu: projectairsim_log().info(f"IMU: {imu} \n"),
        )

        # Subscribe to the Drone's Magnetometer sensor and print the magnetometer data
        client.subscribe(
            drone.sensors["Magnetometer"]["magnetometer"],
            lambda _, magnetometer: projectairsim_log().info(
                f"Magnetometer: {magnetometer} \n"
            ),
        )

        # Subscribe to the Drone's Airspeed sensor and print the airspeed data
        client.subscribe(
            drone.sensors["Airspeed"]["airspeed"],
            lambda _, airspeed: projectairsim_log().info(f"Airspeed: {airspeed} \n"),
        )

        keep_logging_sensors()

    except KeyboardInterrupt:
        projectairsim_log().info("Stopping sensor logging...\n")

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()

        image_display.stop()
