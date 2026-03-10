from os.path import dirname, realpath
import sys
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
from core.CanJointMotor import CanJointMotor
from core.CanScrewMotor import CanScrewMotor
import time
import math

# This test will hold a position for a certain amount of time


if __name__ == "__main__":
  core.CANHelper.init("can0")

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  joint1 = CanJointMotor(can0, 0x141) #ID: 1
  joint2 = CanJointMotor(can0, 0x142) #ID: 2

  print('Starting')
  
  try:
    time.sleep(2)

    for i in range(600):
      print(i)
      joint1.pos_ctrl(2)
      time.sleep(1)  
  except(KeyboardInterrupt) as e:
    print(e)
    joint1.motor_stop()    
    joint2.motor_stop()

  joint1.motor_stop()    
  joint2.motor_stop()

  core.CANHelper.cleanup("can0")
