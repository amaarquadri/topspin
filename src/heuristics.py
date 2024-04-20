import math
from config import Config

config = Config()
    
def time_to_contact(x, v):
    t = (v[2] + math.sqrt(v[2]**2 + 2 * config.g * (config.z_paddle - x[2]))) / config.g
    return t

def get_point_of_contact(x, v):
    t = time_to_contact(x, v)
    x = x + v[0] * t
    y = y + v[1] * t
    z = z + v[2] * t - 0.5 * config.g * t * t
    
    vz = vz[2] - config.g * t

    return t, [x, y, z], [v[0], v[1], vz]

def get_heuristics(t, x, x_f, v_f):
    """
    Input:
    current ball pose (x)
    ball pose at point of contact (x_f)
    ball velocity at point of contact (v_f)
    """
    roll = max(config.roll_k * (x[0] - config.x_des), config.roll_max)
    pitch = max(config.pitch_k * (x[1] - config.y_des), config.pitch_max)

    v_ball_des = math.sqrt(2 * config.g * config.z_des)
    vz_des = v_ball_des / config.e + v_f[2]

    x_des = [x_f[0], x_f[1], x_f[2]]
    theta_des = [roll, pitch, 0.0]
    v_des = [0.0, 0.0, vz_des]

    return t, x_des, theta_des, v_des

def run(x, v):
    t, x_f, v_f = get_point_of_contact(x, v)
    t, x_des, theta_des, v_des = get_heuristics(t, x, x_f, v_f) # garbage theta[2], v[0], v[1]

    return t, x_des, theta_des, v_des