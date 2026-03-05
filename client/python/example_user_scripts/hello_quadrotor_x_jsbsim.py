"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates flying a JSBSim quadrotor_x drone with camera sensors.
"""

import asyncio

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.image_utils import ImageDisplay
from projectairsim.utils import projectairsim_log


def set_jsbsim_attributes(drone: Drone) -> None:
    jsbsim_attributes = {
        "fcs/esc-cmd-norm[0]": 0.55,
        "fcs/esc-cmd-norm[1]": 0.55,
        "fcs/esc-cmd-norm[2]": 0.55,
        "fcs/esc-cmd-norm[3]": 0.55,
    }

    for attr_name, attr_value in jsbsim_attributes.items():
        try:
            drone.set_jsbsim_property(attr_name, attr_value)
            readback = drone.get_jsbsim_property(attr_name)
            projectairsim_log().info(
                f"JSBSim atributo aplicado: {attr_name}={readback:.3f}"
            )
        except Exception as err:
            projectairsim_log().warning(
                f"No se pudo aplicar atributo JSBSim {attr_name}: {err}"
            )


async def main():
    client = ProjectAirSimClient()
    image_display = ImageDisplay()

    try:
        client.connect()

        world = World(client, "scene_quadrotor_x_jsbsim.jsonc", delay_after_load_sec=2)
        drone = Drone(client, world, "Drone1")

        # chase_cam_window = "ChaseCam"
        # image_display.add_chase_cam(chase_cam_window)
        # client.subscribe(
        #     drone.sensors["Chase"]["scene_camera"],
        #     lambda _, chase: image_display.receive(chase, chase_cam_window),
        # )

        # rgb_name = "RGB-Image"
        # image_display.add_image(rgb_name, subwin_idx=0)
        # client.subscribe(
        #     drone.sensors["DownCamera"]["scene_camera"],
        #     lambda _, rgb: image_display.receive(rgb, rgb_name),
        # )

        # depth_name = "Depth-Image"
        # image_display.add_image(depth_name, subwin_idx=2)
        # client.subscribe(
        #     drone.sensors["DownCamera"]["depth_camera"],
        #     lambda _, depth: image_display.receive(depth, depth_name),
        # )

        # image_display.start()

        # drone.enable_api_control()
        set_jsbsim_attributes(drone)
        # # drone.arm()

        # projectairsim_log().info("takeoff_async: starting")
        # takeoff_task = await drone.takeoff_async()
        # await takeoff_task
        # projectairsim_log().info("takeoff_async: completed")

        # move_up_task = await drone.move_by_velocity_async(
        #     v_north=0.0, v_east=0.0, v_down=-1.0, duration=4.0
        # )
        # await move_up_task

        # move_down_task = await drone.move_by_velocity_async(
        #     v_north=0.0, v_east=0.0, v_down=1.0, duration=4.0
        # )
        # await move_down_task

        # projectairsim_log().info("land_async: starting")
        # land_task = await drone.land_async()
        # await land_task
        # projectairsim_log().info("land_async: completed")

        # drone.disarm()
        # drone.disable_api_control()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        client.disconnect()
        image_display.stop()


if __name__ == "__main__":
    asyncio.run(main())
