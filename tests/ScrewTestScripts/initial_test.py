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

import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    # Initilize Can Bus
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    # Initialize Motor
    print("Trying to initialize motors")
    screwMotor = CanUJoint(can0, 1, 6, MIN_POS = 0 * 2 * 3.14, MAX_POS = 10 * 2 * 3.14)
    print('Motor initialization complete')
    
    # Move Motor
    input('Press Enter to spin screw motors')
    speed_input = -5
    screwMotor.speed_ctrl(speed_input)

    # Stop Motor
    input('Press Enter to stop screw motors')
    screwMotor.motor_off()
    print('Done')

    # Can Bus Cleanup
    core.CANHelper.cleanup("can0")