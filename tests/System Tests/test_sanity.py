import os
import can
import numpy as np
import time

from os.path import dirname, realpath  
import sys  
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)  

import core.CANHelper
from core.CanScrewMotor import CanScrewMotor
from core.CanUJoint import CanUJoint
from core.CanArduinoSensors import CanArduinoSensors

if __name__ == "__main__":
  core.CANHelper.init("can0")
  
  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
  # deviceId = 0x01
  # sensor = CanArduinoSensors(can0, deviceId)

  joint1 = CanUJoint(can0, 0, 6)
  joint1.motor_stop() # Allow for user to rotate motor by hand 

  try:
    for _ in range(1000):
      # print(f"joint1 multiturn_position: {joint1.read_multiturn_position()} joint1 position: {joint1.read_singleturn_position()}")
      print(joint1.read_multiturn_position())
      # joint1.read_DIY_multiturn_position()
      # print(sensor.readHumidityAndTemperature())
      time.sleep(0.1)
  except (KeyboardInterrupt, ValueError) as e: # Kill with ctrl + c
    print(e)

  # screw1.motor_stop()
  joint1.motor_stop()
  #joint2.motor_stop()

  core.CANHelper.cleanup("can0")