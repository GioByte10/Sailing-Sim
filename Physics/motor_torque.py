import numpy as np
from motor_command_state import MotorCommand

def compute_motor_torques(tau, sail_lift, sail_drag, haptic_state, params):

    motor_cmd = MotorCommand()

    # Wheel Torque
    motor_cmd.wh_torque = tau[5] / params.steering_ratio
    
    # Winch Torque
    sheet_tension = np.sqrt(sail_lift**2 + sail_drag**2) # sail force magnitude
    motor_cmd.wi_torque = sheet_tension * params.winch_radius

    return motor_cmd