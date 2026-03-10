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
from core.CanMiniJointMotor import CanJointMotor
from core.CanMiniScrewMotor import CanScrewMotor

if __name__ == "__main__":
  core.CANHelper.init("can0")

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  joint_bend = CanJointMotor(can0, 0x149) #ID: 9

  (temp,volt,err) = joint_bend.read_motor_err_and_voltage()
  print('Motor Temp: ',temp)
  print('Motor Volt: ', volt)

  print('\nStarting Test:\n')
  time.sleep(1)

  start_pos = joint_bend.read_multiturn_position()
  print('Start position: ', start_pos)

  set_pos = 50
  slp = 3

  for i in range(2):
    joint_bend.pos_ctrl(set_pos)
    time.sleep(slp)
    print('Position 1: ', joint_bend.read_multiturn_position())
    joint_bend.pos_ctrl(-set_pos)
    time.sleep(slp)
    print('Position 2: ', joint_bend.read_multiturn_position())
    joint_bend.pos_ctrl(0)
    time.sleep(slp)

  end_pos = joint_bend.read_multiturn_position()
  print('End Position: ', end_pos)

  try:
    time.sleep(1)
  except(KeyboardInterrupt) as e:
    print(e)  

  joint_bend.motor_stop()    
  
  print('\nTest Done')

  core.CANHelper.cleanup("can0")
