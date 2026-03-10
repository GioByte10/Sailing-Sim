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

if __name__ == "__main__":
  core.CANHelper.init("can0")

  read_pos_joint = []
  read_speeds_joint = []
  read_torque_joint = []
  read_voltage_joint = []
  read_temp_joint = []

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  joint_bend = CanJointMotor(can0, 0x141) #ID: 1
  joint_bend.motor_stop()    

  print('Starting Test:')

  time.sleep(2)

  joint_bend.speed_ctrl(10)
  for i in range(100*60):
    (joint_torque, joint_speed, joint_m_pos) = joint_bend.read_motor_status()
    # joint_m_pos = joint_bend.read_multiturn_position()
    read_pos_joint.append(joint_m_pos)
    read_speeds_joint.append(joint_speed)
    read_torque_joint.append(joint_torque)

    (joint_temp, joint_volt, joint_err) = joint_bend.read_motor_err_and_voltage()
    read_voltage_joint.append(joint_volt)
    read_temp_joint.append(joint_temp)

    time.sleep(.01)
  
  try:
    time.sleep(1)
  except(KeyboardInterrupt) as e:
    print(e)
    joint_bend.motor_stop()    

  joint_bend.motor_stop()    
  
  end_mark = 5
  del read_pos_joint[0:end_mark]
  del read_speeds_joint[0:end_mark]
  del read_torque_joint[0:end_mark]
  del read_voltage_joint[0:end_mark]
  del read_temp_joint[0:end_mark]

  torque_mean = [np.mean(read_torque_joint)]*len(read_torque_joint)
  print('Average torque for test: ', torque_mean[0])

  print('Test Done')
  print('Loading Plots...')
  fig, axs = plt.subplots(5)
  axs[0].plot(read_pos_joint,'b-')
  axs[0].set_title('Position (Multiloop)')
  axs[1].plot(read_speeds_joint,'r-')
  axs[1].set_title('Speed')
  axs[2].plot(read_torque_joint,'g-')
  axs[2].set_title('Torque')
  axs[2].plot(torque_mean,'k-')
  axs[3].plot(read_temp_joint,'k-')
  axs[3].set_title('Temp')
  axs[4].plot(read_voltage_joint,'c-')
  axs[4].set_title('Voltage')
  plt.show()

  core.CANHelper.cleanup("can0")
