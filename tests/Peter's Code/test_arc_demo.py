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

# This will run a dual-sine wave on both axis.

if __name__ == "__main__":
  core.CANHelper.init("can0")

  read_pos_joint = []
  read_speeds_joint = []
  read_torque_joint = []
  end_pos = []

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  joint1 = CanJointMotor(can0, 0x141) #ID: 1
  joint2 = CanJointMotor(can0, 0x142) #ID: 2
  screw1 = CanScrewMotor(can0, 0x143) #ID: 3

  print('Starting Test:')

  time.sleep(2)

  amp = 1

  screw1.speed_ctrl(10)
  for i in range(10000):
    print(i)
    joint1.pos_ctrl(amp*math.pi*math.sin(2*math.pi*i/10000))
    joint2.pos_ctrl(amp*math.pi*math.cos(2*math.pi*i/10000))
   
    (joint_torque, joint_speed, joint_s_pos) = joint1.read_motor_status()
    joint_m_pos = joint1.read_multiturn_position()
    read_pos_joint.append(joint_m_pos)
    read_speeds_joint.append(joint_speed)
    read_torque_joint.append(joint_torque)

    time.sleep(0.001)
  
  try:
    time.sleep(1)
  except(KeyboardInterrupt) as e:
    print(e)
    joint1.motor_stop()    
    joint2.motor_stop()
    screw1.motor_stop()


  joint1.motor_stop()    
  joint2.motor_stop()
  screw1.motor_stop()
  
  print('Test Done')
  fig, axs = plt.subplots(3)
  axs[0].plot(read_pos_joint,'b-')
  axs[0].set_title('Position (Multiloop)')
  axs[1].plot(read_speeds_joint,'r-')
  axs[1].set_title('Speed')
  axs[2].plot(read_torque_joint,'g-')
  axs[2].set_title('Torque')
  plt.show()

  core.CANHelper.cleanup("can0")
