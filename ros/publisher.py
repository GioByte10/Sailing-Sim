import rclpy

from rclpy.node import Node

from std_msgs.msg import String

import can
from core.CanMotor import CanMotor 
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor

core.CANHelper.init("can0") # Intiailize can0
can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan') # Create can bus object 

testMotor = CanUJoint(can0, 0, 1) # Initialize motor with can bus object 

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Motor Speed: {testMotor.read_speed()}"    

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    testMotor.motor_stop()

    print('Done')

    core.CANHelper.cleanup("can0")


if __name__ == '__main__':
    main()