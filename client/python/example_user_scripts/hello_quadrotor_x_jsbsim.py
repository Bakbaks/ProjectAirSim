"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates flying a JSBSim quadrotor_x drone with camera sensors.
"""

import asyncio
import contextlib

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


def _get_jsbsim_float(drone: Drone, prop_name: str) -> float:
    try:
        return float(drone.get_jsbsim_property(prop_name))
    except Exception:
        return float("nan")


async def stream_ground_truth_kinematics(
    drone: Drone, sample_period_sec: float = 0.1
) -> None:
    ft_to_m = 0.3048

    while True:
        gt = drone.get_ground_truth_kinematics()

        timestamp_ns = gt.get("timestamp", -1)
        pose = gt.get("pose", {})
        position = pose.get("position", {})
        linear_velocity = gt.get("twist", {}).get("linear", {})
        linear_acceleration = gt.get("accelerations", {}).get("linear", {})

        projectairsim_log().info(
            "GT t=%s | pos=(%.3f, %.3f, %.3f) | vel=(%.3f, %.3f, %.3f) | acc=(%.3f, %.3f, %.3f)",
            str(timestamp_ns),
            float(position.get("x", 0.0)),
            float(position.get("y", 0.0)),
            float(position.get("z", 0.0)),
            float(linear_velocity.get("x", 0.0)),
            float(linear_velocity.get("y", 0.0)),
            float(linear_velocity.get("z", 0.0)),
            float(linear_acceleration.get("x", 0.0)),
            float(linear_acceleration.get("y", 0.0)),
            float(linear_acceleration.get("z", 0.0)),
        )

        udot_ft = _get_jsbsim_float(drone, "accelerations/udot-ft_sec2")
        vdot_ft = _get_jsbsim_float(drone, "accelerations/vdot-ft_sec2")
        wdot_ft = _get_jsbsim_float(drone, "accelerations/wdot-ft_sec2")

        projectairsim_log().info(
            "JSBSim acc body udot/vdot/wdot = (%.3f, %.3f, %.3f) ft/s^2 | (%.3f, %.3f, %.3f) m/s^2",
            udot_ft,
            vdot_ft,
            wdot_ft,
            udot_ft * ft_to_m,
            vdot_ft * ft_to_m,
            wdot_ft * ft_to_m,
        )

        await asyncio.sleep(sample_period_sec)


async def main():
    client = ProjectAirSimClient()
    image_display = ImageDisplay()

    try:
        client.connect()

        world = World(client, "scene_quadrotor_x_jsbsim.jsonc", delay_after_load_sec=2)
        drone = Drone(client, world, "Drone1")

        projectairsim_log().info(
            "Monitoreando ground-truth continuamente. Presiona Ctrl+C para detener."
        )
        gt_task = asyncio.create_task(
            stream_ground_truth_kinematics(drone, sample_period_sec=0.1)
        )

        try:
            await gt_task
        finally:
            gt_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await gt_task

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
        # set_jsbsim_attributes(drone)
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
