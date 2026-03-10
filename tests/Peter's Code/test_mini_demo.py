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
from core.CanMotor import CanMotor
from core.CanScrewMotor import CanScrewMotor
import time
import math

if __name__ == "__main__":
  core.CANHelper.init("can0")

  read_pos_joint = []
  read_speeds_joint = []
  read_torque_joint = []
  end_pos = []

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  joint_bend = CanJointMotor(can0, 0x149) #ID: 9
  joint_twist = CanJointMotor(can0, 0x14A) #ID: 10
  screw1 = CanJointMotor(can0, 0x14B) #ID: 11
  screw2 = CanJointMotor(can0, 0x14C) #ID: 12

  print('Starting Test:')

  time.sleep(5)

  screw1.speed_ctrl(-2)
  screw2.speed_ctrl(2)
  for i in range(20000):
    joint_bend.pos_ctrl(3.1415*40*math.sin(2*math.pi*i/10000))
    joint_twist.pos_ctrl(3.1415*40*math.sin(2*math.pi*i/10000))
   
    (joint_torque, joint_speed, joint_s_pos) = joint_bend.read_motor_status()
    joint_m_pos = joint_bend.read_multiturn_position()
    read_pos_joint.append(joint_m_pos)
    read_speeds_joint.append(joint_speed)
    read_torque_joint.append(joint_torque)

    time.sleep(0.001)
  
  try:
    time.sleep(1)
  except(KeyboardInterrupt) as e:
    print(e)
    joint_bend.motor_stop()    
    joint_twist.motor_stop()
    screw1.motor_stop()
    screw2.motor_stop()


  joint_bend.motor_stop()    
  joint_twist.motor_stop()
  screw1.motor_stop()
  screw2.motor_stop()
  
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
