from doctest import testmod
import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor

from os.path import dirname, realpath  
import sys
from core.CanMotor import CanMotor  
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)  

import math as m

print("Enter motor ID (integer, starting at 1) : ")
motor_id = input()
if motor_id:
  motor_id = int(motor_id)-1
else :
  motor_id = 0
  print("Defaulting to ID = 0")

print("Enter actuator gear ratio: ")
gear_ratio = input()
if gear_ratio:
  gear_ratio = float(gear_ratio)
else :
  gear_ratio = 11
  print("Defaulting to gear ratio = 100")

print("Enter MINIMUM actuator angle (revolutions): ")
min_pos = input()
if min_pos:
  min_pos = float(min_pos)
else :
  min_pos = -.24
  print("Defaulting to min pos = -0.25")

print("Enter MAX actuator angle (revolutions): ")
max_pos = input()
if max_pos:
  max_pos = float(max_pos)
else :
  max_pos = .23
  print("Defaulting to min pos = 0.25")

print("For position control, enter MAX speed (rev/s): ")
max_speed = input()
if max_speed:
  max_speed = float(max_speed)
else :
  max_speed = .5
  print("Defaulting to max speed = 1")

if __name__ == "__main__":
  core.CANHelper.init("can0") # Intiailize can0
  can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan') # Create can bus object 

  testMotor = CanUJoint(can0, motor_id, gear_ratio, MIN_POS = min_pos * 2 * m.pi, MAX_POS = max_pos * 2 * m.pi) # Initialize motor with can bus object 

  print("Enter Desired Control method")
  print("1 = Position Control (Rotations)")
  print("2 = Velocity Control (Rotations Per Second)")
  print("3 = Torque Control (Amps)")

  controlMethod = 0

  controlMethod = int(input())
   
  try:
    while True: 
      # print("Motor Temp: ", testMotor.read_temp())
      # print("Enter command value:")
      val = float(input())
      if controlMethod == 1:
        testMotor.pos_ctrl(val * 2 * m.pi, max_speed) 
      elif controlMethod == 2:
        testMotor.speed_ctrl(val * 2 * m.pi)
      elif controlMethod == 3:
        testMotor.torque_ctrl(val)
      else:
        raise KeyboardInterrupt

  except(KeyboardInterrupt) as e:
    print(e)

  testMotor.motor_stop()

  print('Done')

  core.CANHelper.cleanup("can0")