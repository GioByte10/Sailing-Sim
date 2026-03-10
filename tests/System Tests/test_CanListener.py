import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))


from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time


if __name__ == "__main__":

	# Initialize CAN bus
	core.CANHelper.init("can0")
	can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

	# Initial CanMotor objects (does not try to send any messages yet)
	motor1 = CanMotor(can0, motor_id = 7, gear_ratio = 1)
	motor_list = [motor1]  # Add all motors to the listener
	motor_listener = MotorListener(motor_list=motor_list)
	input("Start Notifier")

	# Start Notifier to listen for messages

	notifier = can.Notifier(can0, [motor_listener])

	# Start motor initialization after user input (this sends start command to motors, etc.)
	input("Initialize motors")
	for motor in motor_list:
		motor.initialize_motor()

	# Wait 1 s, then start periodic sends after user input
	time.sleep(1)
	input("Start periodic sends")

	# # Start speed control while also reading and printing motor data
	command_speed = 5
	motor1.read_status_periodic()
	time.sleep(0.1)
	motor1.read_multiturn_periodic()
	time.sleep(0.1)
	initial_pos = motor1.motor_data.multiturn_position
	print("Initial position: ", initial_pos)

	input("Start position control")
	motor1.initialize_control_command()
	motor1.set_control_mode("speed", command_speed)

	try:
		while True:
			print(motor1.motor_data)
			time.sleep(0.01)
			pass


	except KeyboardInterrupt:
		motor1.stop_all_tasks()
		motor1.motor_off()
		core.CANHelper.cleanup("can0")
		can0.shutdown()
		print("Exiting")
		exit(0)
