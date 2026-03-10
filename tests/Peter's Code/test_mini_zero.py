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
  
  read_pos_joint = []
  read_speeds_joint = []
  read_torque_joint = []
  end_pos = []

  (temp,volt,err) = joint_bend.read_motor_err_and_voltage()
  print('Motor Temp: ',temp)
  print('Motor Volt: ', volt)

  print('\nStarting Zeroing:\n')
  time.sleep(1)

  start_pos = joint_bend.read_multiturn_position()
  print('Start position: ', start_pos)

  for k in range(2):
    print('k = ', k)
    v = (.1)*(-1)**(k+1)
    print('Velocity = ', v)
    temp_torque = 0
    i = 1
    while temp_torque < .3:
      print("i = ", i)
      i = i+1
      (joint_torque, joint_speed, joint_s_pos) = joint_bend.read_motor_status()
      joint_m_pos = joint_bend.read_multiturn_position()
      joint_bend.speed_ctrl(v)
      time.sleep(0.001)

      temp_torque = abs(joint_torque)
      print('Current Torque: ', temp_torque)
      print('Current Position: ', joint_m_pos)

      print('------------------')

      read_pos_joint.append(joint_m_pos)
      read_speeds_joint.append(joint_speed)
      read_torque_joint.append(joint_torque)
    
    joint_bend.motor_stop() 
    end_pos.append(joint_m_pos)   
    print(end_pos)
    print('\nMoving back\n')
    time.sleep(2)
    if k == 0:
      joint_bend.pos_ctrl(start_pos)
      time.sleep(5)

  print('The end points are: ', end_pos)

  # time.sleep(2)
  # print('Going to endpoints')
  # joint_bend.pos_ctrl(end_pos[0]*.95)

  try:
    time.sleep(1)
  except(KeyboardInterrupt) as e:
    print(e)
    print('STOPPING')
    joint_bend.motor_stop()    

  joint_bend.motor_stop()    
  
  print('\nTest Done')
  print('Loading plot')

  fig, axs = plt.subplots(3)
  axs[0].plot(read_pos_joint,'b-')
  axs[0].set_title('Position (Multiloop)')
  axs[1].plot(read_speeds_joint,'r-')
  axs[1].set_title('Speed')
  axs[2].plot(read_torque_joint,'g-')
  axs[2].set_title('Torque')
  plt.show()

  core.CANHelper.cleanup("can0")
