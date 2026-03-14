import numpy as np

# a helper function
def compute_apparent_wind(boat_vel_body, env, boat_yaw):
    """
    Computes apparent velocities into boat frame
    
    Inputs: 
        boat_state
        wind_vel vector of [speed, earth direction]
        params

    Returns:
        apparent_wind speed
        apparent wind angle
    """
    
    # computes apparent wind in boat body frame
    # boat_vel_body: np.array([u, v, w]) in body frame
    # wind_vel_earth: np.array([vx, vy, vz]) in earth frame
    # boat yaw 

    wind_speed, wind_dir = env.wind_field
    # print(wind_speed)
    # print(wind_dir)
    # convert wind from speed + direction to earth-frame vector
    wind_earth = np.array([
        wind_speed * np.cos(wind_dir),  # x-component
        wind_speed * np.sin(wind_dir),  # y-component
        0                               # assume horizontal wind
    ])

    c, s = np.cos(boat_yaw), np.sin(boat_yaw)
    R = np.array([[c, s, 0],
                  [-s, c, 0],
                  [0, 0, 1]])
    
    wind_body = R @ wind_earth
    apparent_wind = wind_body- boat_vel_body
    a = np.arctan2(apparent_wind[1], apparent_wind[0]) # angle between boat heading and apparent wind in xy plane

    return apparent_wind, a   
