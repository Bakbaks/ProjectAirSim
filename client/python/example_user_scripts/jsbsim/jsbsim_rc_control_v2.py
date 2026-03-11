"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates controlling a JSBSim fixed-wing aircraft in Project AirSim with a gamepad.
This script implements standard RC controls for fixed-wing aircraft:
- Left stick: Roll (horizontal), Pitch (vertical)
- Right stick: Yaw (horizontal), Throttle (vertical)

This is similar to PX4 remote control mapping for fixed-wing aircraft.
"""

import argparse
from enum import Enum
import time

from projectairsim import ProjectAirSimClient, World, Drone
from projectairsim.utils import projectairsim_log

try:
	from inputs import get_gamepad, UnpluggedError
except ImportError as err:
	raise ImportError(
		"The 'inputs' package is required. Install with: pip install inputs"
	) from err

try:
	import msvcrt
	import sys
	HAS_MSVCRT = True
except ImportError:
	HAS_MSVCRT = False


JOYSTICK_MIN = -32768.0
JOYSTICK_MAX = 32767.0
DEADZONE = 0.05  # Ignore inputs smaller than 5% of range


class GamepadButtonEvent:
	"""Enum for gamepad button events."""
	class Button(Enum):
		A = "BTN_SOUTH"
		B = "BTN_EAST"
		X = "BTN_WEST"
		Y = "BTN_NORTH"
		LB = "BTN_TL"
		RB = "BTN_TR"
		BACK = "BTN_SELECT"
		START = "BTN_START"


def normalize_signed_axis(value: int) -> float:
	"""Normalize a signed joystick axis value to [-1.0, 1.0]."""
	normalized = float(value) / 32768.0
	# Apply deadzone
	if abs(normalized) < DEADZONE:
		return 0.0
	# Apply soft exponential curve for more responsive control
	sign = 1 if normalized > 0 else -1
	normalized_abs = abs(normalized)
	# Exponential curve: makes small movements more precise
	curved = sign * (normalized_abs ** 1.3)
	return max(-1.0, min(1.0, curved))


def normalize_throttle(value: int) -> float:
	"""Normalize throttle from [-1.0, 1.0] to [0.0, 1.0]."""
	signed = normalize_signed_axis(value)
	# Map from [-1, 1] to [0, 1]
	return max(0.0, min(1.0, (signed + 1.0) * 0.5))


class ControlMode(Enum):
	"""Control modes for the RC."""
	DIRECT = "direct"  # Direct control: aileron, elevator, rudder, throttle
	STABILIZED = "stabilized"  # Stabilized mode: uses autopilot for attitude hold
	HEADING_HOLD = "heading-hold"  # Heading hold mode


class RemoteControlReader:
	"""Manages input from an Xbox-compatible game controller or keyboard fallback."""

	def __init__(self, use_keyboard: bool = False):
		self.use_keyboard = use_keyboard
		self.gamepad_found = False
		self.input_state = {
			# Analog axes
			"joystick_LH": 0,  # Left stick horizontal - Roll (aileron)
			"joystick_LV": 0,  # Left stick vertical - Pitch (elevator)
			"joystick_RH": 0,  # Right stick horizontal - Yaw (rudder)
			"joystick_RV": 0,  # Right stick vertical - Throttle
			# Digital buttons
			"button_A": False,
			"button_B": False,
			"button_X": False,
			"button_Y": False,
			"button_LB": False,
			"button_RB": False,
			"button_back": False,
			"button_start": False,
		}
		self.button_pressed = {}  # Track button state changes
		
		if not use_keyboard:
			# Try to detect gamepad on startup
			try:
				events = get_gamepad()
				if events:
					self.gamepad_found = True
					projectairsim_log().info("Gamepad detected successfully")
			except UnpluggedError:
				logger = projectairsim_log()
				logger.warning("No gamepad found!")
				logger.warning("Solutions:")
				logger.warning("  1. For DualShock 4: Use DS4Windows https://github.com/Jays2Kings/DS4Windows")
				logger.warning("  2. Ensure your controller is plugged in and recognized by Windows")
				logger.warning("  3. Use keyboard control with: --use-keyboard flag")
				self.use_keyboard = True
				logger.info("Switching to keyboard control...")

	def _read_keyboard(self):
		"""Read keyboard input as a fallback."""
		# This is a simple implementation using keyboard arrow keys
		# W/S for pitch, A/D for roll, Arrow up/down for throttle, Arrow left/right for yaw
		if not HAS_MSVCRT:
			time.sleep(0.01)  # Prevent busy wait
			return self.input_state
		
		rate = 5000.0  # Simulated joystick range per key press
		try:
			if msvcrt.kbhit():
				key = ord(msvcrt.getch())
				
				# W/S for pitch (elevator)
				if key == ord('w'):
					self.input_state["joystick_LV"] = min(32767, self.input_state["joystick_LV"] + rate)
				elif key == ord('s'):
					self.input_state["joystick_LV"] = max(-32768, self.input_state["joystick_LV"] - rate)
				
				# A/D for roll (aileron)
				elif key == ord('a'):
					self.input_state["joystick_LH"] = max(-32768, self.input_state["joystick_LH"] - rate)
				elif key == ord('d'):
					self.input_state["joystick_LH"] = min(32767, self.input_state["joystick_LH"] + rate)
				
				# Q/E for throttle
				elif key == ord('q'):
					self.input_state["joystick_RV"] = max(-32768, self.input_state["joystick_RV"] - rate)
				elif key == ord('e'):
					self.input_state["joystick_RV"] = min(32767, self.input_state["joystick_RV"] + rate)
				
				# Z/C for yaw (rudder)
				elif key == ord('z'):
					self.input_state["joystick_RH"] = max(-32768, self.input_state["joystick_RH"] - rate)
				elif key == ord('c'):
					self.input_state["joystick_RH"] = min(32767, self.input_state["joystick_RH"] + rate)
			else:
				# Decay input values gradually when no key is pressed
				decay = 0.95
				self.input_state["joystick_LH"] *= decay
				self.input_state["joystick_LV"] *= decay
				self.input_state["joystick_RH"] *= decay
				self.input_state["joystick_RV"] *= decay if self.input_state["joystick_RV"] < 0 else decay
				time.sleep(0.01)
		except Exception:
			pass
		
		return self.input_state

	def read(self):
		"""Read gamepad or keyboard input and update input state."""
		if self.use_keyboard:
			return self._read_keyboard()
		
		updated_input = False
		retry_count = 0
		max_retries = 3

		while not updated_input and retry_count < max_retries:
			try:
				events = get_gamepad()
				if not events and retry_count == 0:
					retry_count += 1
					time.sleep(0.01)
					continue
					
				for event in events:
					if event.ev_type == "Absolute":
						# Analog stick inputs
						if event.code == "ABS_X":
							self.input_state["joystick_LH"] = event.state
							updated_input = True
						elif event.code == "ABS_Y":
							self.input_state["joystick_LV"] = event.state
							updated_input = True
						elif event.code == "ABS_RX":
							self.input_state["joystick_RH"] = event.state
							updated_input = True
						elif event.code == "ABS_RY":
							self.input_state["joystick_RV"] = event.state
							updated_input = True
					elif event.ev_type == "Key":
						# Button inputs
						if event.code == "BTN_SOUTH":  # A button
							self.input_state["button_A"] = event.state == 1
							updated_input = True
						elif event.code == "BTN_EAST":  # B button
							self.input_state["button_B"] = event.state == 1
							updated_input = True
						elif event.code == "BTN_WEST":  # X button
							self.input_state["button_X"] = event.state == 1
							updated_input = True
						elif event.code == "BTN_NORTH":  # Y button
							self.input_state["button_Y"] = event.state == 1
							updated_input = True
						elif event.code == "BTN_TL":  # LB button
							self.input_state["button_LB"] = event.state == 1
							updated_input = True
						elif event.code == "BTN_TR":  # RB button
							self.input_state["button_RB"] = event.state == 1
							updated_input = True
						elif event.code == "BTN_SELECT":  # Back button
							self.input_state["button_back"] = event.state == 1
							updated_input = True
						elif event.code == "BTN_START":  # Start button
							self.input_state["button_start"] = event.state == 1
							updated_input = True
					
					if updated_input:
						break
				
				if not updated_input:
					updated_input = True  # Exit loop even if no input to prevent hanging
					
			except UnpluggedError:
				retry_count += 1
				if retry_count >= max_retries:
					logger = projectairsim_log()
					logger.error("Gamepad disconnected! Switching to keyboard control...")
					self.use_keyboard = True
					return self._read_keyboard()
				time.sleep(0.01)

		return self.input_state


class StandardRCController:
	"""Standard RC controller for fixed-wing aircraft."""

	def __init__(self, drone: Drone, control_mode: ControlMode = ControlMode.DIRECT):
		self.drone = drone
		self.control_mode = control_mode
		self.logger = projectairsim_log()
		self.last_control_time = 0
		self.is_armed = False  # Motor arming state
		self.throttle_override = 0.0  # When disarmed, override throttle to 0
		self.trim_values = {
			"aileron": 0.0,
			"elevator": 0.0,
			"rudder": 0.0,
			"throttle": 0.5,
		}

	def arm_motors(self):
		"""Arm the motors for flight."""
		if not self.is_armed:
			self.is_armed = True
			self.logger.info("🚀 MOTORS ARMED - Ready for takeoff!")
			# Optionally set a minimum throttle level for motor spin-up
			self.drone.set_jsbsim_property("fcs/throttle-cmd-norm", 0.05)

	def disarm_motors(self):
		"""Disarm the motors - cut all power."""
		if self.is_armed:
			self.is_armed = False
			self.logger.warning("⛔ MOTORS DISARMED")
			# Cut throttle immediately
			self.drone.set_jsbsim_property("fcs/throttle-cmd-norm", 0.0)
		self.throttle_override = 0.0

	def handle_button_input(self, input_state):
		"""Handle button inputs for arming/disarming and other functions."""
		# START button (☰ menu button on DualShock) - ARM MOTORS
		if input_state["button_start"]:
			self.arm_motors()
		
		# BACK button (< button on DualShock) - DISARM MOTORS
		if input_state["button_back"]:
			self.disarm_motors()
		
		# X button - Reset trim
		if input_state["button_X"]:
			self.trim_values = {
				"aileron": 0.0,
				"elevator": 0.0,
				"rudder": 0.0,
				"throttle": 0.5,
			}
			self.logger.info("Trim reset to neutral")
		
		# Y button - Increase elevator trim (nose up)
		if input_state["button_Y"]:
			self.trim_values["elevator"] = min(0.2, self.trim_values["elevator"] + 0.01)
		
		# A button - Decrease elevator trim (nose down)
		if input_state["button_A"]:
			self.trim_values["elevator"] = max(-0.2, self.trim_values["elevator"] - 0.01)

	def apply_direct_control(self, input_state):
		"""
		Apply direct control mode.
		Maps gamepad inputs directly to control surfaces.
		Standard mapping:
		- Left stick horizontal (LH): Roll (aileron)
		- Left stick vertical (LV): Pitch (elevator)
		- Right stick horizontal (RH): Yaw (rudder)
		- Right stick vertical (RV): Throttle
		"""
		# Handle button input first
		self.handle_button_input(input_state)

		# Get normalized control inputs
		roll_cmd = normalize_signed_axis(input_state["joystick_LH"])
		pitch_cmd = -normalize_signed_axis(input_state["joystick_LV"])  # Negative for intuitive pitch
		yaw_cmd = normalize_signed_axis(input_state["joystick_RH"])
		throttle_cmd = normalize_throttle(input_state["joystick_RV"])

		# If disarmed, force throttle to 0
		if not self.is_armed:
			throttle_cmd = 0.0

		# Apply trim values
		aileron_cmd = roll_cmd + self.trim_values["aileron"]
		elevator_cmd = pitch_cmd + self.trim_values["elevator"]
		rudder_cmd = yaw_cmd + self.trim_values["rudder"]
		throttle_final = throttle_cmd + self.trim_values["throttle"] - 0.5  # Center trim

		# Clamp to valid ranges
		aileron_cmd = max(-1.0, min(1.0, aileron_cmd))
		elevator_cmd = max(-1.0, min(1.0, elevator_cmd))
		rudder_cmd = max(-1.0, min(1.0, rudder_cmd))
		throttle_final = max(0.0, min(1.0, throttle_final))

		# Set JSBSim properties
		self.drone.set_jsbsim_property("fcs/throttle-cmd-norm", throttle_final)
		self.drone.set_jsbsim_property("fcs/aileron-cmd-norm", aileron_cmd)
		self.drone.set_jsbsim_property("fcs/elevator-cmd-norm", elevator_cmd)
		self.drone.set_jsbsim_property("fcs/rudder-cmd-norm", rudder_cmd)

		return {
			"roll": roll_cmd,
			"pitch": pitch_cmd,
			"yaw": yaw_cmd,
			"throttle": throttle_cmd,
			"armed": self.is_armed,
		}

	def apply_stabilized_control(self, input_state):
		"""
		Apply stabilized mode using autopilot attitude hold.
		The autopilot maintains the desired attitude while pilot controls roll/pitch angles.
		"""
		# Handle button input first
		self.handle_button_input(input_state)

		# Get normalized control inputs
		roll_cmd = normalize_signed_axis(input_state["joystick_LH"])
		pitch_cmd = -normalize_signed_axis(input_state["joystick_LV"])
		yaw_rate_cmd = normalize_signed_axis(input_state["joystick_RH"])
		throttle_cmd = normalize_throttle(input_state["joystick_RV"])

		# If disarmed, force throttle to 0
		if not self.is_armed:
			throttle_cmd = 0.0

		# Convert roll/pitch commands to angles (in degrees)
		# Maximum ±30 degrees bank/pitch angle
		max_bank_angle = 30.0
		max_pitch_angle = 20.0
		
		bank_setpoint = roll_cmd * max_bank_angle
		pitch_setpoint = pitch_cmd * max_pitch_angle

		# Set throttle
		self.drone.set_jsbsim_property("fcs/throttle-cmd-norm", throttle_cmd)

		# Enable attitude hold mode with AP
		# Note: These properties are defined in c310_rc_ap.xml
		self.drone.set_jsbsim_property("ap/attitude_hold", 1.0)
		
		# For this simple implementation, we can still use direct aileron/elevator commands
		# but the autopilot will help stabilize
		aileron_cmd = (roll_cmd + self.trim_values["aileron"]) * 0.7  # Reduced authority
		elevator_cmd = (pitch_cmd + self.trim_values["elevator"]) * 0.7  # Reduced authority
		
		aileron_cmd = max(-1.0, min(1.0, aileron_cmd))
		elevator_cmd = max(-1.0, min(1.0, elevator_cmd))

		self.drone.set_jsbsim_property("fcs/aileron-cmd-norm", aileron_cmd)
		self.drone.set_jsbsim_property("fcs/elevator-cmd-norm", elevator_cmd)
		self.drone.set_jsbsim_property("fcs/rudder-cmd-norm", yaw_rate_cmd)

		return {
			"roll_setpoint": bank_setpoint,
			"pitch_setpoint": pitch_setpoint,
			"yaw_rate": yaw_rate_cmd,
			"throttle": throttle_cmd,
			"armed": self.is_armed,
		}

	def apply_heading_hold(self, input_state):
		"""
		Apply heading hold mode using autopilot.
		Pilot controls pitch and heading, autopilot maintains wing level with heading.
		"""
		# Handle button input first
		self.handle_button_input(input_state)

		# Get normalized control inputs
		pitch_cmd = -normalize_signed_axis(input_state["joystick_LV"])
		yaw_cmd = normalize_signed_axis(input_state["joystick_RH"])
		throttle_cmd = normalize_throttle(input_state["joystick_RV"])

		# If disarmed, force throttle to 0
		if not self.is_armed:
			throttle_cmd = 0.0

		# Set throttle
		self.drone.set_jsbsim_property("fcs/throttle-cmd-norm", throttle_cmd)

		# Enable autopilot heading hold
		self.drone.set_jsbsim_property("ap/heading_hold", 1.0)
		
		# Set heading command (convert yaw input [-1,1] to heading change rate)
		heading_rate = yaw_cmd * 30.0  # Max ±30 degrees per second
		self.drone.set_jsbsim_property("ap/heading-comm", heading_rate)

		# Set pitch command
		pitch_setpoint = pitch_cmd * 20.0  # Max ±20 degrees pitch
		elevator_cmd = normalize_signed_axis(input_state["joystick_LV"]) * 0.5
		self.drone.set_jsbsim_property("fcs/elevator-cmd-norm", elevator_cmd)

		return {
			"pitch_cmd": pitch_cmd,
			"heading_rate": heading_rate,
			"throttle": throttle_cmd,
			"armed": self.is_armed,
		}

	def control(self, input_state, print_debug=True):
		"""Apply control based on current mode."""
		current_time = time.time()
		
		if self.control_mode == ControlMode.DIRECT:
			control_info = self.apply_direct_control(input_state)
		elif self.control_mode == ControlMode.STABILIZED:
			control_info = self.apply_stabilized_control(input_state)
		elif self.control_mode == ControlMode.HEADING_HOLD:
			control_info = self.apply_heading_hold(input_state)
		else:
			control_info = self.apply_direct_control(input_state)

		if print_debug and (current_time - self.last_control_time) > 0.5:  # Print every 0.5s
			mode_name = self.control_mode.value
			self._print_control_status(control_info, mode_name)
			self.last_control_time = current_time

	def _print_control_status(self, control_info, mode_name):
		"""Print control status for debugging."""
		print(f"\n[{mode_name}] ", end="")
		for key, value in control_info.items():
			print(f"{key}={value:7.3f} ", end="")
		print()


def control_loop(
	controller: StandardRCController,
	rc_reader: RemoteControlReader,
	print_rc: bool = True,
):
	"""Main control loop."""
	input_state = rc_reader.read()
	controller.control(input_state, print_debug=print_rc)


def parse_args():
	parser = argparse.ArgumentParser(
		description="Control a JSBSim fixed-wing aircraft in ProjectAirSim using a gamepad."
	)
	parser.add_argument(
		"mode",
		nargs="?",
		choices=["direct", "stabilized", "heading-hold"],
		default="direct",
		help="Control mode: direct (direct surface control), stabilized (attitude hold), heading-hold",
	)
	parser.add_argument("--address", type=str, default="127.0.0.1")
	parser.add_argument("--topicsport", type=int, default=8989)
	parser.add_argument("--servicesport", type=int, default=8990)
	parser.add_argument("--sceneconfigfile", type=str, default="scene_cessna310_rc.jsonc")
	parser.add_argument("--simconfigpath", type=str, default="sim_config/")
	parser.add_argument("--robot", type=str, default="c310")
	parser.add_argument("--delay", type=int, default=2)
	parser.add_argument("--use-keyboard", action="store_true", help="Use keyboard control instead of gamepad")
	return parser.parse_args()


def main():
	args = parse_args()
	
	# Map string mode to ControlMode enum
	mode_map = {
		"direct": ControlMode.DIRECT,
		"stabilized": ControlMode.STABILIZED,
		"heading-hold": ControlMode.HEADING_HOLD,
	}
	control_mode = mode_map.get(args.mode, ControlMode.DIRECT)
	
	logger = projectairsim_log()
	logger.info(f"Starting RC control - Mode: {control_mode.value}")
	if args.use_keyboard:
		logger.info("Using keyboard control:")
		logger.info("  W/S: Pitch (elevator)")
		logger.info("  A/D: Roll (aileron)")
		logger.info("  Q/E: Throttle")
		logger.info("  Z/C: Yaw (rudder)")
	else:
		logger.info(f"Gamepad mapping:")
		logger.info(f"  Left stick (horizontal): Roll (aileron)")
		logger.info(f"  Left stick (vertical): Pitch (elevator)")
		logger.info(f"  Right stick (horizontal): Yaw (rudder)")
		logger.info(f"  Right stick (vertical): Throttle")

	client = ProjectAirSimClient(
		address=args.address,
		port_topics=args.topicsport,
		port_services=args.servicesport,
	)
	rc_reader = RemoteControlReader(use_keyboard=args.use_keyboard)

	try:
		client.connect()
		world = World(
			client=client,
			scene_config_name=args.sceneconfigfile,
			delay_after_load_sec=args.delay,
			sim_config_path=args.simconfigpath,
		)
		drone = Drone(client, world, args.robot)
		
		# Create controller
		controller = StandardRCController(drone, control_mode=control_mode)

		drone.enable_api_control()
		logger.info("API control enabled. Starting control loop...")

		while True:
			control_loop(controller, rc_reader, print_rc=True)

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
