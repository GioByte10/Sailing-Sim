import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from typing import List # For type hinting

import can
from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
from core.CANHelper import init, cleanup
from core.CanArduinoSensors import CanArduinoSensors
import time

GEAR_RATIO = 11

class Segment(object):
    '''
        Class that holds 3 motors for each segment of the snake
    '''
    def __init__(self, canBus:can.ThreadSafeBus, uJointID_1:int = None, 
                 screwMotorID:int = None, uJointID_2:int = None):
        # TODO: Please switch from CanMotor to UJoint and ScrewMotor classes by 
        #   setting the correct gear ratio
        self.motor_names = []
        self.motors = []
        if uJointID_1 is not None:
            self.motor_names.append('uJoint1')
            self.motors.append(CanMotor(canBus, uJointID_1, GEAR_RATIO))
        if screwMotorID is not None:
            self.motor_names.append('screw')
            self.motors.append(CanMotor(canBus, screwMotorID, 1))
        if uJointID_2 is not None:
            self.motor_names.append('uJoint2')
            self.motors.append(CanMotor(canBus, uJointID_2, GEAR_RATIO))

        # Start listening for messages
        self.motor_listener = MotorListener(motor_list=self.motors)

        # Initialize motors here
        for motor in self.motors:
            motor.initialize_motor()
            time.sleep(0.1)

    def get_pos(self):
        '''
            Returns the current position of the segment
        '''
        return [motor.motor_data.multiturn_position for motor in self.motors]

    def get_speed(self):
        '''
            Returns the current speed of the segment
        '''
        return [motor.motor_data.speed for motor in self.motors]
    
    def get_torque(self):
        '''
            Returns the current torque of the segment
        '''
        return [motor.motor_data.torque for motor in self.motors]
    
    def get_name(self):
        '''
            Returns the name of the segment
        '''
        return self.motor_names

    def start(self):
        '''
            Starts/resumes all motors in the segment
        '''
        for motor in self.motors:
            motor.motor_resume()

            # Start reading periodic messages
            motor.read_status_periodic(0.15)
            motor.read_multiturn_periodic(0.15)
            motor.read_motor_state_periodic(0.15)
            motor.initialize_control_command(0.15)


    def stop(self):
        '''
            Stops all motors in the segment
        '''
        for motor in self.motors:
            motor.motor_stop()
            
            # Turn off periodic messages
            motor.stop_all_tasks()

    def off(self):
        '''
            Turns off all motors in the segment
        '''
        for motor in self.motors:
            motor.motor_off()

            # Turn off periodic messages
            motor.stop_all_tasks()

    def pos_ctrl(self, pos:float, motor_name:str):
        '''
            Sets the position of the motor with the given name
        '''
        self.motors[self.motor_names.index(motor_name)].set_control_mode("position", pos)

    def speed_ctrl(self, speed:float, motor_name:str):
        '''
            Sets the speed of the motor with the given name
        '''
        self.motors[self.motor_names.index(motor_name)].set_control_mode("speed", speed)

    def torque_ctrl(self, torque:float, motor_name:str):
        '''
            Sets the torque of the motor with the given name
        '''
        self.motors[self.motor_names.index(motor_name)].set_control_mode("torque", torque)

class ARCSnakeROS(Node):
    '''
        Joint angle information for each segment of the snake
    '''
    def __init__(self, list_of_segments: List[Segment], time_period: float = 0.1,
                 list_of_can_arduino_sensors: List[CanArduinoSensors] = None):
        super().__init__('arcsnake')
        self.list_of_segments = list_of_segments
        self.publisher_ = self.create_publisher(JointState, '/arcsnake/joints_current', 10)
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.joint_subscription = self.create_subscription(
            JointState,
            'arcsnake/joints_desired',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.state_subscription = self.create_subscription(
            String,
            'arcsnake/state',
            self.state_listener_callback,
            10
        )
        self.state_subscription # prevent unused variable warning

        # IMU Publishers initialization
        # self.temp_publishers = []
        # if list_of_can_arduino_sensors:
        #     for can_arduino_sensor in list_of_can_arduino_sensors:
        #         self.temp_publishers.append()

        self.get_logger().info("ARCSnakeROS node initialized")

    def listener_callback(self, msg):
        '''
        RIGHT NOW ONLY LISTENS TO POSITION COMMANDS FOR U_JOINTS and VELOCITY COMMANDS FOR SCREW MOTORS
        THIS IS NOT IDEAL AND WE SHOULD COME UP WITH A BETTER SOLUTION.
        '''
        for idx, joint_name in enumerate(msg.name):
            # Get segment number from msg name
            seg_num = int(joint_name.split('_')[0][3:])

            # Get motor name (e.g. screw, U-Joint1, U-Joint2) from msg name
            motor_name = joint_name.split('_')[1]

            if motor_name == 'screw':
                self.list_of_segments[seg_num].speed_ctrl(msg.velocity[idx], motor_name)
            else:
                # WARNING: THE TORQUE OVERRIDES POSITOIN CONTROL IF IT IS NOT ZERO
                if msg.effort[idx] != 0:
                    self.list_of_segments[seg_num].torque_ctrl(msg.effort[idx], motor_name)
                else:
                    self.list_of_segments[seg_num].pos_ctrl(msg.position[idx], motor_name)

    def state_listener_callback(self, msg):
        if msg.data == 'stop':
            for segment in self.list_of_segments:
                segment.stop()
        elif msg.data == "off":
            for segment in self.list_of_segments:
                segment.off()
        elif msg.data == "start":
            for segment in self.list_of_segments:
                segment.start()
        else:
            self.get_logger().info("Invalid state command")

    def timer_callback(self):
        msg = JointState()

        # Append segment data to message
        for idx, segment in enumerate(self.list_of_segments):
            for name in segment.get_name():
                msg.name.append("seg{}_{}".format(idx, name))
            for pos in segment.get_pos():
                msg.position.append(pos)
            for vel in segment.get_speed():
                msg.velocity.append(vel)
            for torque in segment.get_torque():
                msg.effort.append(torque)
        self.publisher_.publish(msg)


def main(args=None):
    # Initialize canbus
    init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    input("Continue")

    # Initialize segments
    segment_list = [ Segment(can0, None, 0, 8),
                     Segment(can0, 10, 1, 5),
                     Segment(can0, 6, 4, 9),
                     Segment(can0, 7, 3, None)]
    
    # Start Notifier to listen for messages
    # THIS LINE OF CODE IS VERY IMPORTANT OTHERWISE THE LISTENERS WILL NOT WORK
    # I.E. THE MOTORS POSITION/VELOCITY WILL NOT BE UPDATED
    can.Notifier(can0, [segment.motor_listener for segment in segment_list])

    # Initialize ROS
    rclpy.init(args=args)
    arcsnake_ros = ARCSnakeROS(segment_list)
    rclpy.spin(arcsnake_ros)

    # Destroy the node
    arcsnake_ros.destroy_node()
    rclpy.shutdown()

    # Turn off motors
    for segment in segment_list:
        segment.stop()

    cleanup("can0")
    can0.shutdown()
    print("Exiting")
    exit(0)


if __name__ == '__main__':
    main()