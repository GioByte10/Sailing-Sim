# Class for motor can listener, which handles received messages and processes them
from can import Listener, Notifier, Message


class MotorListener(Listener):
	# Initialize the listener with a list of CanMotor objects
	def __init__(self, motor_list=None):
		self.motors = {}
		if motor_list is not None:
			self.set_motor_list(motor_list)

	def set_motor_list(self, motor_list):
		'''
		Sets the list of CanMotor objects to process messages for

		Args:
			motor_list: list of CanMotor objects
		'''
		for motor in motor_list:
			self.motors[motor.id] = motor	
	
	def on_message_received(self, msg):
		'''
		Called everytime Notifier notifies Listener object of a new message
		'''
		# print("Listener received message ", msg)
		# Filter by arbitration ID
		if msg.arbitration_id in self.motors.keys():
			self.motors[msg.arbitration_id].process_message(msg)
		
		elif msg.arbitration_id == 4:
			pass
		
		else:
			pass
			#print(f"~~~~~Received message from unknown motor: {msg.arbitration_id}~~~~~~~~~")


			
