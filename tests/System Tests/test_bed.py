from asyncore import loop
from audioop import mul
from cProfile import run
from doctest import run_docstring_examples
from os.path import dirname, realpath
from pdb import post_mortem
import sys
from unittest import mock

from more_itertools import sample
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
from core.timeout import TimeoutError

import math
import numpy as np
import time
    
if __name__ == "__main__":
    # Initilize Can Bus
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    # Initialize Motors
    print("Trying to initialize motors")
    #finScrew = CanMotor(can0, 0, 1)
    linVelMotor = CanMotor(can0, 2, 1)
    print('Motor initialization complete')

    # Initialize Linear Velocity Reading
    speedData = []; radToLin = 9.6 / (2*math.pi) # cm/s

    # Move Motor via Speed Control
    input('Press Enter to spin screw motors')
    '''
        negative is common notation for forward but it DEPENDS on fin orientation
        10 is a good speed for standard testing
    '''
    #finScrew.speed_ctrl(-10)

    try:
        while True:
            speed = linVelMotor.read_speed() * radToLin # cm/s
            speedData.append(speed)
            time.sleep(1/125) # resolution of 125 Hz
    finally:
        # Write Out and Save Sensor Data
        speedArray = np.array(speedData)
        filename = 'test_data.npy'
        np.save(filename, speedArray)

        # Stop Motor
        #finScrew.motor_off()
        linVelMotor.motor_off()
        print('Done')

        # Can Bus Cleanup
        core.CANHelper.cleanup("can0")
        exit(0)