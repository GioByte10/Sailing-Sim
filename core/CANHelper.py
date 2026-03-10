import os

def init(CANBUSID):
  os.system(f'sudo ip link set dev {CANBUSID} down')
  os.system(f'sudo ip link set {CANBUSID} type can bitrate 1000000')
  os.system(f'sudo ip link set dev {CANBUSID} up')
  os.system(f'sudo ip link set {CANBUSID} txqueuelen 100')

def cleanup(CANBUSID):
  os.system(f'sudo ip link set dev {CANBUSID} down')

