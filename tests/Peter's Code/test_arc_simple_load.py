from doctest import run_docstring_examples
from os.path import dirname, realpath
from pdb import post_mortem
import sys
from unittest import mock
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


# This code is meant to run a sine wave and spit out the plots.

if __name__ == "__main__":
  core.CANHelper.init("can0")

  read_s_pos_joint = []
  read_m_pos_joint = []
  read_speeds_joint = []
  read_torque_joint = []
  t_vec = []

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  gear_ratio = 11
  joint1 = CanMotor(can0, 1, gear_ratio) #ID: 1
  # joint2 = CanMotor(can0, 1, gear_ratio) #ID: 2
  # screw1 = CanScrewMotor(can0, 0x143) #ID: 3

  joint1.motor_start()

  print('Starting Test:')

  time.sleep(1)

  zero_pos = 0.1
  amp = .2 * m.pi
  loop_rate = 1000 #Hz
  run_time = 10 #seconds

  for i in range(run_time*loop_rate):
    t_start = time.time()
    print("Time: ", '{0:.2f}'.format(i/loop_rate), " seconds")
    joint1.pos_ctrl(amp*m.sin(2*m.pi*i/10000)+zero_pos,.5)
    
    (joint_torque, joint_speed, joint_s_pos) = joint1.read_motor_status()
    # joint_m_pos = joint1.read_multiturn_position()
    joint_m_pos = 0
    read_s_pos_joint.append(joint_s_pos)
    read_m_pos_joint.append(joint_m_pos)
    read_speeds_joint.append(joint_speed)
    read_torque_joint.append(joint_torque)

    t_end = time.time()
    t_execute = t_end - t_start
    t_vec.append(1/t_execute)
    # print("Time to execute = ", t_execute)
    # print(1/t_execute)
    time.sleep(abs(1/loop_rate-t_execute))
  
    try:
      time.sleep(abs(1/loop_rate-t_execute))
    except(KeyboardInterrupt) as e:
      print(e)
      joint1.motor_stop()    
      # joint2.motor_stop()
      # screw1.motor_stop()
      break

  joint1.motor_stop()    
  # joint2.motor_stop()
  # screw1.motor_stop()

  print('Test Done')

  cut_amount = 150
  del read_s_pos_joint[1:cut_amount]
  del read_m_pos_joint[1:cut_amount]
  del read_speeds_joint[1:cut_amount]
  del read_torque_joint[1:cut_amount]

  def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y,box,mode='same')
    return y_smooth

  fig, axs = plt.subplots(5)
  axs[0].plot(read_s_pos_joint,'b-')
  axs[0].set_title('Position (Singleloop)')
  axs[1].plot(read_m_pos_joint,'m-')
  axs[1].set_title('Position (Multiloop)')
  axs[2].plot(smooth(read_speeds_joint,10),'r-')
  axs[2].set_title('Speed')
  axs[3].plot(smooth(read_torque_joint,200),'g-')
  axs[3].set_title('Torque')
  axs[4].plot(t_vec)
  axs[4].set_title('Execute time (Hz)')
  plt.show()

  core.CANHelper.cleanup("can0")
