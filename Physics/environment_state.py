import numpy as np

class Environment:
    def __init__(self):
        self.wind_field = (5, 0)
        self.ocean_field = (0,0)
        
        #self.wind_field = lambda pos, t: (5.0, 0.0) # wind speed m/s, wind direction earth frame
        #self.ocean_field = lambda pos, t: (5.0, 0.0) # current speed m/s, direction earth frame

    def set_env_from_pos(self, pos):
        """
        Sets environment wind and current based on position

        Inputs: 
            pos: [x y] position vector
        
        """
        # TODO some wind correlation
        self.wind_field = (0, 0)
        self.ocean_field = (0, 0)


