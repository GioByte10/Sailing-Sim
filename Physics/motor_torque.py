import numpy as np
from motor_command_state import MotorCommand

def compute_motor_torques(boat_state, tau, sail_lift, sail_drag, haptic_state, params):

    motor_cmd = MotorCommand()

    # Wheel Torque
    motor_cmd.wh_torque = ((90 - boat_state.nu[5]) * tau[5]) / params.steering_ratio
    print(f"Tau {tau[5]}")
    print("Motor Torques:", motor_cmd.wh_torque)
    
    # Winch Torque
    sheet_tension = np.sqrt(sail_lift**2 + sail_drag**2) # sail force magnitude
    motor_cmd.wi_torque = sheet_tension * params.winch_radius #/ params.winch_ratio

    return motor_cmd