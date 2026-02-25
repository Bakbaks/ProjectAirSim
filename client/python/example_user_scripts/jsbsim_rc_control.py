"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates controlling a JSBSim aircraft in Project AirSim with a gamepad.
"""

import argparse
from enum import Enum

from projectairsim import ProjectAirSimClient, World, Drone
from projectairsim.utils import projectairsim_log

try:
	from inputs import get_gamepad
except ImportError as err:
	raise ImportError(
		"The 'inputs' package is required. Install with: pip install inputs"
	) from err


JOYSTICK_MIN = -32768.0
JOYSTICK_MAX = 32767.0


def normalize_signed_axis(value: int) -> float:
	return max(-1.0, min(1.0, float(value) / 32768.0))


def normalize_throttle(value: int) -> float:
	signed = normalize_signed_axis(value)
	return max(0.0, min(1.0, (signed + 1.0) * 0.5))


class RemoteControlReader:
	"""Manages input from an Xbox-compatible game controller."""

	def __init__(self):
		self.input_state = {
			"joystick_RH": 0,
			"joystick_RV": 0,
			"joystick_LH": 0,
			"joystick_LV": 0,
		}

	def read(self):
		updated_input = False

		while not updated_input:
			events = get_gamepad()
			for event in events:
				recognized_event = True
				if event.ev_type == "Absolute":
					if event.code == "ABS_X":
						self.input_state["joystick_LH"] = event.state
					elif event.code == "ABS_Y":
						self.input_state["joystick_LV"] = event.state
					elif event.code == "ABS_RX":
						self.input_state["joystick_RH"] = event.state
					elif event.code == "ABS_RY":
						self.input_state["joystick_RV"] = event.state
					else:
						recognized_event = False
				else:
					recognized_event = False

				if recognized_event:
					updated_input = True

		return self.input_state


class ControlType(Enum):
	RPY = "mode1"  # Roll, Pitch, Yaw
	TCS = "mode2"  # Turn rate, climb rate, speed


def apply_rpy_mode(drone: Drone, input_state):
	throttle = normalize_throttle(input_state["joystick_RV"])
	roll = normalize_signed_axis(input_state["joystick_RH"])
	pitch = normalize_signed_axis(input_state["joystick_LV"])
	yaw = normalize_signed_axis(input_state["joystick_LH"])

	drone.set_jsbsim_property("fcs/throttle-cmd-norm[0]", throttle)
	drone.set_jsbsim_property("fcs/throttle-cmd-norm[1]", throttle)
	drone.set_jsbsim_property("fcs/aileron-cmd-norm", roll)
	drone.set_jsbsim_property("fcs/elevator-cmd-norm", pitch)
	drone.set_jsbsim_property("fcs/rudder-cmd-norm", yaw)


def apply_tcs_mode(drone: Drone, input_state):
	throttle = normalize_throttle(input_state["joystick_RV"])
	turn_cmd = normalize_signed_axis(input_state["joystick_LH"])
	climb_cmd = normalize_signed_axis(input_state["joystick_LV"])

	drone.set_jsbsim_property("fcs/throttle-cmd-norm[0]", throttle)
	drone.set_jsbsim_property("fcs/throttle-cmd-norm[1]", throttle)
	drone.set_jsbsim_property("ap/heading-comm", turn_cmd)
	drone.set_jsbsim_property("ap/climb-rate-cmd", climb_cmd)


def control_loop(
	drone: Drone,
	user_mode: ControlType,
	rc_reader: RemoteControlReader,
	print_rc: bool = True,
):
	input_state = rc_reader.read()

	if print_rc:
		print(
			f"Left=({input_state['joystick_LH']:6n},{input_state['joystick_LV']:6n}), "
			f"Right=({input_state['joystick_RH']:6n},{input_state['joystick_RV']:6n}), ",
			end="\r",
		)

	if user_mode == ControlType.RPY:
		apply_rpy_mode(drone, input_state)
	elif user_mode == ControlType.TCS:
		apply_tcs_mode(drone, input_state)


def parse_args():
	parser = argparse.ArgumentParser(
		description="Controla un avión JSBSim en ProjectAirSim usando gamepad."
	)
	parser.add_argument(
		"mode",
		nargs="?",
		choices=[ControlType.RPY.value, ControlType.TCS.value],
		default=ControlType.RPY.value,
		help="mode1=RPY, mode2=TCS",
	)
	parser.add_argument("--address", type=str, default="127.0.0.1")
	parser.add_argument("--topicsport", type=int, default=8989)
	parser.add_argument("--servicesport", type=int, default=8990)
	parser.add_argument("--sceneconfigfile", type=str, default="scene_cessna310_rc.jsonc")
	parser.add_argument("--simconfigpath", type=str, default="sim_config/")
	parser.add_argument("--robot", type=str, default="c310")
	parser.add_argument("--delay", type=int, default=2)
	return parser.parse_args()


def main():
	args = parse_args()
	user_mode = ControlType(args.mode)
	projectairsim_log().info(f"Control mode selected: {user_mode.value}")

	client = ProjectAirSimClient(
		address=args.address,
		port_topics=args.topicsport,
		port_services=args.servicesport,
	)
	rc_reader = RemoteControlReader()

	try:
		client.connect()
		world = World(
			client=client,
			scene_config_name=args.sceneconfigfile,
			delay_after_load_sec=args.delay,
			sim_config_path=args.simconfigpath,
		)
		drone = Drone(client, world, args.robot)

		drone.enable_api_control()

		while True:
			control_loop(drone, user_mode, rc_reader)

	except KeyboardInterrupt:
		print("\nExiting...")
	finally:
		try:
			drone.disable_api_control()
		except Exception:
			pass
		client.disconnect()


if __name__ == "__main__":
	main()
