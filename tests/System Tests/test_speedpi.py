import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor

from os.path import dirname, realpath  
import sys
from core.CanMotor import CanMotor  
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)  

import matplotlib.pyplot as plt
import time

time_data   = []
vel_data = []
desiredSpeed = 4 

if __name__ == "__main__":
  core.CANHelper.init("can0") # Intiailize can0
  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan') # Create can bus object 

  testMotor = CanUJoint(can0, 2, 1) # Initialize motor with can bus object 

  print(testMotor.read_motor_pid())
  testMotor.override_PI_values(100, 100, 255, 30, 50, 50)
  print(testMotor.read_motor_pid())
  testMotor.speed_ctrl(desiredSpeed)
  initialTime = time.time()
  count = 0
  try:
    while True: 
        time_data.append(time.time() - initialTime)
        
        vel_data.append(testMotor.read_speed())

        count += 1
        if (count > 25):
          time_data.pop(0)
          vel_data.pop(0)


        plt.plot(time_data, vel_data, color = "black") 
        plt.draw()
        plt.pause(0.01)

  except(KeyboardInterrupt) as e:
    print(e)

  testMotor.motor_stop()

  print('Done')

  core.CANHelper.cleanup("can0")