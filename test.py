import numpy as np
import math
import time

def speed_ctrl_rampup(to_rad, v_rate, max_speed=(1890*2*math.pi)/60):
        """Set speed in rad/s but ramp up at specified rate
        -

        Args:
            to_rad (RAD): Set speed
            v_rate (rad/s^2): rate at which to increase speed until hitting set speed
            max_speed (RAD/s, optional): Set max allowable speed. Defaults to 20*2*math.pi.
            **06/20/23 Note: 1890 rpm max given by motor spec sheet, tachometer test in lab (with motor "4") confirms it is close to 1800 rpm max

        Returns:
            RAD/s: Returns current speed
        """        
        if to_rad > max_speed:
            to_rad = max_speed
        if to_rad < -max_speed:
            to_rad = -max_speed

        # num_steps = 100 # may want to make this an argument?
        t_loop = 0.05 # Setting this as a constant so it doesn't cause issues by being too small
        num_steps = int(to_rad / (v_rate * t_loop))
        # speed_increment = to_rad / num_steps
        t0 = time.time()
        for i in range(num_steps):
            cur_t = time.time() - t0

            set_speed = v_rate * t_loop * (i+1)
            # set_speed_dps = self.gear_ratio * 100 * self.utils.radToDeg(set_speed)
            # byte1, byte2, byte3, byte4 = self.utils.int_to_bytes(int(set_speed_dps), 4)
            # msg = self.send([0xa2, 0x00, 0x00, 0x00, byte4, byte3, byte2, byte1], wait_for_response=True)
            print(set_speed, cur_t)

            # set loop rate
            # Ensures proper loop rate
            t_execute = time.time() - cur_t - t0 # Time it took to execute code inside loop
            if t_execute >= t_loop: # If loop takes longer to execute than expected loop time, loop rate is too fast
                err_count = err_count + 1
                print('Loop rate too fast.')
            elif t_execute < t_loop: # Otherwise wait difference between executed time and expected loop time
                time.sleep(t_loop - t_execute)

        # final command to get up to final set speed (since we rounded down for num_steps)
        # set_speed_dps = self.gear_ratio * 100 * self.utils.radToDeg(to_rad)
        # byte1, byte2, byte3, byte4 = self.utils.int_to_bytes(int(set_speed_dps), 4)
        # msg = self.send([0xa2, 0x00, 0x00, 0x00, byte4, byte3, byte2, byte1], wait_for_response=True)
        print(to_rad, time.time()-t0)
        # return self.utils.degToRad(self.utils.readBytesList([msg.data[5], msg.data[4]])) / self.gear_ratio


speed_ctrl_rampup(to_rad=10, v_rate=2)