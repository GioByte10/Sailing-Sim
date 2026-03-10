from asyncore import loop
from audioop import mul
from cProfile import run
from doctest import run_docstring_examples
from os.path import dirname, realpath
from pdb import post_mortem
import sys
from unittest import mock

from more_itertools import sample
arcsnake_v2_path = dirname(dirname(realpath(__file__)))
sys.path.append(arcsnake_v2_path)

import os
import can
import math as m
import numpy as np
import time
import matplotlib.pyplot as plt
from datetime import datetime
from pyinstrument import Profiler

import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanMotor import CanMotor
from core.CanScrewMotor import CanScrewMotor

### Meant for debugging message sending/receiving

if __name__ == "__main__":
	core.CANHelper.init("can0")
	can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

	motor_id = [10, 0]
	gear_ratio = 11
	
	print("Trying to initialize motors")
	motor1 = CanMotor(can0, motor_id[0], gear_ratio=11)
	motor2 = CanMotor(can0, motor_id[1], gear_ratio=1)
	print('Motor initialization complete')

	input("Press Enter to read zero position.")
	zero_pos1 = motor1.read_multiturn_position()
	zero_pos2 = motor2.read_multiturn_position()
	# print(f"Zero (Multiturn) Position = {zero_pos1}")

	# Run commands in loop, with variable frequency, print/saving message information
	command_rate = 100 # Hz, frequency with which to send motor commands
	run_time = 5 # s

	input("Press Enter to start command loop.")
	t_start = time.time()
	cur_time = time.time()-t_start
	while cur_time <= run_time:
		cur_time = time.time() - t_start

		# Commands
		motor1.pos_ctrl(zero_pos1)
		motor2.pos_ctrl(zero_pos2)
		# time.sleep(1/command_rate)
		motor1.read_multiturn_position()
		motor2.read_multiturn_position()
		# time.sleep(1/command_rate)
		(torque, speed, s_pos) = motor1.read_motor_status()
		(torque, speed, s_pos) = motor2.read_motor_status()
		# time.sleep(1/command_rate)

	motor1.motor_stop()
	motor2.motor_stop()
	motor1.save_message_log(f"/home/myeoh/Documents/GitHub/arcsnake_v2/tests/System Tests/TestCanMessages/message_log_rateCONT_postfix_m1.csv")
	motor2.save_message_log(f"/home/myeoh/Documents/GitHub/arcsnake_v2/tests/System Tests/TestCanMessages/message_log_rateCONT_postfix_m2.csv")
	motor1.motor_off()
	motor2.motor_off()

	core.CANHelper.cleanup("can0")

