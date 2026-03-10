import os
import can
import math
from .CanUtils import CanUtils
from .timeout import timeout
import time

class CanArduinoSensors(object):
  def __init__(self, bus, arduino_id):
    self.canBus = bus
    self.utils = CanUtils()
    self.id = arduino_id
    self.wakeup_time = time.time()

  @timeout(1)
  def _send(self, data, send_retries=0):
    send_msg = can.Message(arbitration_id=self.id, data=data, is_extended_id=False, is_rx=False, timestamp = time.time() - self.wakeup_time)
    self.canBus.send(send_msg)

    num_recv_retries = 0
    while True:
      # Checking canbus message recieved with keyboard interrupt saftey
      try:
        # print("trying to receive")
        recv_msg = self.canBus.recv()
        if recv_msg.arbitration_id == self.id:
          break
        num_recv_retries += 1
      except (KeyboardInterrupt, ValueError) as e:
        print(e)
        break
    return recv_msg
  
  def send(self, data, num_time_outs=100):
    '''
        Wrapper for _send that retries if timeout occurs.
    '''
    for i in range(num_time_outs):
        try:
            # print(f'Trying to send message {hex(data[0])}')
            return self._send(data, send_retries=i)
        except TimeoutError:
            continue
    raise TimeoutError("No response")

  def readHumidityAndTemperature(self):
    '''
      returns humidity, temperature, pressure
    '''
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    return int(recv_msg.data[-1]), int(recv_msg.data[-2]), int(recv_msg.data[-3]*100)

  def readImuCalibrationAndTemp(self):
    '''
      returns system calib, gyro calib, accel_calib, mag_calib, imu temperature in Celsius
      calib ranges from 0 (bad) to 3 (best)
    '''
    data = [1, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    return recv_msg.data[1], recv_msg.data[2], recv_msg.data[3], recv_msg.data[4], recv_msg.data[5]

  def readImuOrientation(self):
    '''
      reads absolute orientation as (x, y, z) Euler angles in degrees
    '''
    data = [2, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    x = self.utils.readBytes(recv_msg.data[2], recv_msg.data[1])
    y = self.utils.readBytes(recv_msg.data[4], recv_msg.data[3])
    z = self.utils.readBytes(recv_msg.data[6], recv_msg.data[5])
    x = x / 16.0
    y = y / 16.0
    z = z / 16.0
    return x, y, z

  def readImuAccelerometer(self):
    '''
      reads accelerometer data (linear acceleration + gravity) in m/s^2
    '''
    data = [3, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    x = self.utils.readBytes(recv_msg.data[2], recv_msg.data[1])
    y = self.utils.readBytes(recv_msg.data[4], recv_msg.data[3])
    z = self.utils.readBytes(recv_msg.data[6], recv_msg.data[5])
    x = x / 100.0
    y = y / 100.0
    z = z / 100.0
    return x, y, z

  def readImuLinearAccel(self):
    '''
      reads linear acceleration (without gravity) in m/s^2
    '''
    data = [4, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    x = self.utils.readBytes(recv_msg.data[2], recv_msg.data[1])
    y = self.utils.readBytes(recv_msg.data[4], recv_msg.data[3])
    z = self.utils.readBytes(recv_msg.data[6], recv_msg.data[5])
    x = x / 100.0
    y = y / 100.0
    z = z / 100.0
    return x, y, z

  def readImuGyroscope(self):
    '''
      reads gyroscope in rad/s
    '''
    data = [5, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    x = self.utils.readBytes(recv_msg.data[2], recv_msg.data[1])
    y = self.utils.readBytes(recv_msg.data[4], recv_msg.data[3])
    z = self.utils.readBytes(recv_msg.data[6], recv_msg.data[5])
    x = x / 16.0
    y = y / 16.0
    z = z / 16.0
    return x, y, z

  def readImuMagnetometer(self):
    '''
      reads magnetometer in microTesla
    '''
    data = [6, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    x = self.utils.readBytes(recv_msg.data[2], recv_msg.data[1])
    y = self.utils.readBytes(recv_msg.data[4], recv_msg.data[3])
    z = self.utils.readBytes(recv_msg.data[6], recv_msg.data[5])
    x = x / 16.0
    y = y / 16.0
    z = z / 16.0
    return x, y, z

  def readImuQuaternion(self):
    '''
      reads orientation in quaternion?
    '''
    data = [7, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    w = self.utils.readBytes(recv_msg.data[1], recv_msg.data[0])
    x = self.utils.readBytes(recv_msg.data[3], recv_msg.data[2])
    y = self.utils.readBytes(recv_msg.data[5], recv_msg.data[4])
    z = self.utils.readBytes(recv_msg.data[7], recv_msg.data[6])
    scale = (1.0 / (1 << 14)) # copied this from internal IMU code
    return scale*w, scale*x, scale*y, scale*z


  def readImuGravity(self):
    '''
      reads gravity vector in m/s^2
    '''
    data = [8, 0, 0, 0, 0, 0, 0, 0]
    recv_msg = self.send(data)
    x = self.utils.readBytes(recv_msg.data[2], recv_msg.data[1])
    y = self.utils.readBytes(recv_msg.data[4], recv_msg.data[3])
    z = self.utils.readBytes(recv_msg.data[6], recv_msg.data[5])
    x = x / 100.0
    y = y / 100.0
    z = z / 100.0
    return x, y, z

  