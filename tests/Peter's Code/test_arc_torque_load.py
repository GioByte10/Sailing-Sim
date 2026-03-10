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


# This code is meant to run a sine wave on one axis 
# and spit out the plot for torque transparency

if __name__ == "__main__":
  core.CANHelper.init("can0")
  
  read_torque_joint = []

  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  gear_ratio = 11
  joint1 = CanMotor(can0, 1, gear_ratio) #ID: 2

  joint1.motor_start()

  print('Starting Test:')

  time.sleep(1)

  zero_pos = 0.1
  amp = 1 * m.pi
  loop_rate = 1000 #Hz
  run_time = 20 #seconds

  joint1.pos_ctrl(zero_pos,1)
  time.sleep(1)

  for i in range(run_time*loop_rate):
    t_start = time.time()
    print("Time: ", '{0:.2f}'.format(i/loop_rate), " seconds")
    joint1.pos_ctrl(amp*m.sin(2*m.pi*i/10000)+zero_pos,1)
    
    (joint_torque, _, _) = joint1.read_motor_status()
    read_torque_joint.append(joint_torque)

    t_end = time.time()
    t_execute = t_end - t_start
    # time.sleep(abs(1/loop_rate-t_execute))
  
    try:
      time.sleep(abs(1/loop_rate-t_execute))
    except(KeyboardInterrupt) as e:
      print(e)
      joint1.motor_stop()    
      break

  joint1.motor_stop()    
  print('Test Done')

  cut_amount = 150
  del read_torque_joint[1:cut_amount]

  def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y,box,mode='same')
    return y_smooth

  # plt.plot(smooth(read_torque_joint,200),'g-')
  plt.plot(read_torque_joint,'g-')
  plt.title("Torque")
  plt.show()

  core.CANHelper.cleanup("can0")
