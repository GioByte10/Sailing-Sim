import os
import can
import math
import time
import matplotlib.pyplot as plt
from datetime import datetime
from pyinstrument import Profiler

from core.CanMotor import CanJointMotor
from core.CanScrewMotor import CanScrewMotor
from core.CanUtils import CanUtils

def init():
    os.system('sudo ifconfig can0 down')
    os.system('sudo ip link set can0 type can bitrate 1000000')
    os.system('sudo ifconfig can0 up')

def cleanup():
    os.system('sudo ifconfig can0 down')

def profile(motor):
    profiler = Profiler()
    profiler.start()

    for i in range(1000):
        motor.pos_ctrl(0x00, 0x41, 0x00, 0x00)
        motor.read_encoder()

    profiler.stop()
    print(profiler.output_text(unicode=True, color=True))

if __name__ == "__main__":
    init()

    # make the can bus
    canBus = can.ThreadSafeBus(channel='can0', bustype='socketcan_ctypes')

    # motor initialization
    joint1 = CanJointMotor(canBus, 0x141)
    joint2 = CanJointMotor(canBus, 0x142)
    screw = CanScrewMotor(canBus, 0x143)
    utils = CanUtils()
    
    start = datetime.now()
    step = []
    # y = []
    # z = []

    for i in range(200):
        try:
            time_since_start = datetime.now() - start
            step.append(time_since_start.total_seconds())
            
            to_vel = (math.pi**2) * math.sin(time_since_start.total_seconds() * 0.2 * math.pi)
            to_pos = (math.pi**2) * math.sin(time_since_start.total_seconds() * 0.2 * math.pi)
            
            joint1.pos_ctrl(to_pos)
            joint_position = joint1.read_position()

            joint2.pos_ctrl(to_pos)
            joint_position = joint2.read_position()

            screw.speed_ctrl(to_vel)
            screw_speed = screw.read_speed()

            loop_dur = datetime.now() - start - time_since_start
            # 10ms for each loop
            time.sleep(max(0, 0.01 - loop_dur.total_seconds()))
        except (KeyboardInterrupt, ValueError) as e:
            print(e)
            break

    joint1.motor_stop()
    joint2.motor_stop()
    screw.motor_stop()
    # plt.plot(x,y,'b-')
    # plt.plot(x,z,'r-')
    
    # plt.xlabel('time (s)')
    # plt.ylabel('speed (rad/s)')
    # plt.legend(["encoder speed", "set speed"])
    # plt.show()

    cleanup()