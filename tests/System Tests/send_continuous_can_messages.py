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


# This code is meant to test simply connecting to motor 

if __name__ == "__main__":
	core.CANHelper.init("can0")
	can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
	gear_ratio = 1

	print("Trying to initialize motors")
	joint1 = CanMotor(can0, 0, gear_ratio)
	joint1.motor_start()
	print("Successfully initialized motors")

	input("Press Enter to start sending CAN messages.")
	while True:
		try:

			print(joint1.read_motor_status())
			time.sleep(1)
		except KeyboardInterrupt as e:
			print(e)
			break

	joint1.motor_off()

	core.CANHelper.cleanup("can0")
