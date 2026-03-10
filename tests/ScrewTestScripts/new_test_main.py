from tests.ScrewTestScripts.Test_Bench_Class import testBench
import time
myTestBench = testBench(30, 2) # screw number, set number
myTestBench.setGroundZero()

myTestBench.startSensorLog()
myTestBench.runSingleSpeedTest({2,3,4,5,6})
# myTestBench.runTorqueTest({.25, .5, .75, 1, 1.25})
myTestBench.stopSensorLog()
