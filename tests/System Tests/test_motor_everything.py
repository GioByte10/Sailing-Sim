import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor

if __name__ == "__main__":
  core.CANHelper.init("can0") # Intiailize can0
  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan') # Create can bus object 

  testMotor = CanUJoint(can0, 7, 1) # Initialize motor with can bus object
  
  print("Enter Desired Control method")
  print("1 = Position Control (Rotations)")
  print("2 = Velocity Control (Rotations Per Second)")
  print("3 = Torque Control (Amps)")

  controlMethod = 0 

  controlMethod = int(input())

  try:
    while True: 
      val = float(input())
      error_flag = testMotor.read_motor_err_and_voltage()[3]
      if error_flag != "No errors":
        print(error_flag)
      if controlMethod == 1:
        testMotor.pos_ctrl(val * 2 * 3.14 ,2) 
      elif controlMethod == 2:
        testMotor.speed_ctrl(val * 2 * 3.14)
      elif controlMethod == 3:
        testMotor.torque_ctrl(val)
      else:
        raise KeyboardInterrupt

  except(KeyboardInterrupt) as e:
    print(e)

  testMotor.motor_stop()

  print('Done')

  core.CANHelper.cleanup("can0")