import time
import numpy as np
from boat_state import BoatState
from environment_state import Environment 
from haptic_state import HapticState
from motor_command_state import MotorCommand
from params import Params
from simulate import run_simulation
#TODO import motor_interface
#TODO import graphics 


def main():
    params = Params()
    boat_state = BoatState()
    haptic_state = HapticState()
    motor_command = MotorCommand()
    env = Environment()

    dt = params.dt
    t = params.t_start
    t_end = params.t_end

    log_state = []
    log_haptic = []
    log_forces = []
    log_torque = []

    # Graphics Start
    # last_draw_time = time.time()
    # draw_interval = 1/60 # for 60 HZ   

    while t < t_end:
        loop_start = time.time()
        print(t)
        # read motors 
        haptic_state.wh[0] = 0.1*np.sin(2*np.pi*.02*t) # wheel position
        haptic_state.wh[1] = 0 # wheel velocity
        haptic_state.wh[2] = 0 # wheel acceleration

        haptic_state.wi[0] = 0.5 #times winch gear ratio winch position
        haptic_state.wi[1] = 0.0 # winch velocity
        haptic_state.wi[2] = 0.0 # winch acceleration

        # simulation for single  step
        boat_state, tau_total, motor_command = run_simulation(boat_state, haptic_state, env, params)

        # Send haptic torques to motors
        wheel_torque = motor_command.wh_torque #tau_total[5] / params.steering_ratio
        winch_torque = motor_command.wi_torque #motor_command.wi_torque

        print(f"Wheel Torque:  {wheel_torque}")
        print(f"Winch Torque: {winch_torque}")
        # motor1_interface.send_torques(wheel_torque)
        # motor2_interface.send_torques(winch_torque)

        # Update Graphics 
        # if (time.time() - last_draw_time) >= draw_interval:
            #graphics.update(boat_state)
            #last_draw_time = time.time()
        
        t += dt # Update time
        time.sleep(dt) # Delay by dt 

        log_state.append(boat_state.as_vector())
        log_haptic.append(haptic_state.as_vector())
        log_forces.append(tau_total)
        log_torque.append([wheel_torque, winch_torque])
    
    log_state=np.array(log_state)
    log_haptic = np.array(log_haptic)
    log_forces = np.array(log_forces)
    log_torque = np.array(log_torque)

    print("Simulation complete")

if __name__ == "__main__":
    main()
