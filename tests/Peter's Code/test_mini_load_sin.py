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

print("Starting")

if __name__ == "__main__":
  core.CANHelper.init("can0")

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

  gear_ratio = 100

  joint_bend = CanMotor(can0, 0, gear_ratio)
  # joint_twist = CanJointMotor(can0, 0x142)

  read_pos_joint = []
  read_speeds_joint = []
  read_torque_joint = []

  amp = math.pi * 1/1.8
  loop_rate = 500 #Hz
  run_time = 20 #seconds

  for i in range(run_time*loop_rate):
    t_start = time.time()
    print("i = ",i)
    joint_bend.pos_ctrl(amp*math.sin(2*math.pi*i/10000),5)
    
    (joint_torque, joint_speed, joint_m_pos) = joint_bend.read_motor_status()
    # joint_m_pos = joint_bend.read_multiturn_position()
    read_pos_joint.append(joint_m_pos)
    read_speeds_joint.append(joint_speed)
    read_torque_joint.append(joint_torque)

    t_end = time.time()
    t_execute = t_end - t_start
    print("Time to execute = ", t_execute)
    print(1/t_execute)
    time.sleep(abs(1/loop_rate-t_execute))
  
    try:
      time.sleep(abs(1/loop_rate-t_execute))
    except(KeyboardInterrupt) as e:
      print(e)
      joint_bend.motor_stop()    
      # joint_twist.motor_stop()
      break

  
  joint_bend.motor_stop()
  # joint_twist.motor_stop()

  
  print('Test Done')
  print('Loading plot')
 
  cut_amount = 100
  del read_pos_joint[1:cut_amount]
  del read_speeds_joint[1:cut_amount]
  del read_torque_joint[1:cut_amount]

  def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y,box,mode='same')
    return y_smooth

  fig, axs = plt.subplots(3)
  axs[0].plot(read_pos_joint,'b-')
  axs[0].set_title('Position (Multiloop)')
  axs[1].plot(smooth(read_speeds_joint,10),'r-')
  axs[1].set_title('Speed')
  axs[2].plot(smooth(read_torque_joint,10),'g-')
  axs[2].set_title('Torque')
  plt.show()



  core.CANHelper.cleanup("can0")