from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
from core.CanArduinoSensors import CanArduinoSensors

if __name__ == "__main__":

	# Initialize CAN bus
	core.CANHelper.init("can0")
	can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
	# can0.set_filters([{"can_id": 0x00, "can_mask": 0xFF, "extended": False}])


	# Initial CanMotor objects (does not try to send any messages yet)
	motor_id = 0
	gear_ratio = 1
	motor1 = CanMotor(can0, motor_id, gear_ratio)
	# motor2 = CanMotor(can0, motor_id=36, gear_ratio=1)
	motor_list = [motor1]  # Add all motors to the listener
	motor_listener = MotorListener(motor_list=motor_list)

	input("Start Notifier")

	# Start Notifier to listen for messages
	can.Notifier(can0, [motor_listener])

	input("Initialize motors")
	# Initialize motors (this sends start command to motors, etc.)
	for motor in motor_list:
		motor.initialize_motor()
	
	time.sleep(.1)
	input("Start periodic torque control")
	motor1.initialize_control_command()
	motor1.set_control_mode("torque", 0.5)
	motor1.read_motor_state_periodic()

	try:
		while True:
			# time.sleep(.1)
			# input("Send single command")
			# motor1.read_status_once()
			target_torque = int(input('Target torque: '))
			motor1.set_control_mode("torque", target_torque)
			print(motor1.motor_data)
			print('\n')
			# time.sleep(1)
			pass
	except KeyboardInterrupt:
		motor1.stop_all_tasks()
		motor1.motor_off()
		core.CANHelper.cleanup("can0")
		can0.shutdown()
		print("Exiting")
		exit(0)


