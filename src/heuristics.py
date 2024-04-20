import math
from config import Config

class Heuristics:
    def __init__(self):
        self.config = Config()

    def get_point_of_contact(self, x, y, z, vx, vy, vz):
        t = (vz + math.sqrt(vz**2 + 2 * self.config.g * (self.config.z_paddle - z))) / self.config.g
        x = vx * t - 0.5 * self.config.g * t * t
        y = vy * t - 0.5 * self.config.g * t * t
        z = vz * t - 0.5 * self.config.g * t * t
        vx = vx - self.config.g * t
        vy = vy - self.config.g * t
        vz = vz - self.config.g * t

        h_max = (vz * vz) / (2 * self.config.g)

        return t, x, y, z, vx, vy, vz, h_max

    def get_heuristics(self, t, x, y, x_f, y_f, z_f, vz_f, h_max):
        x_des = x_f
        y_des = y_f
        r_des = max(self.config.roll_k * (x - self.config.x_des), self.config.roll_max)
        p_des = max(self.config.pitch_k * (y - self.config.y_des), self.config.pitch_max)

        v_ball_des = math.sqrt(2 * self.config.g * h_max)
        vz_des = v_ball_des / self.config.e + vz_f

        return t, x_des, y_des, z_f, r_des, p_des, 0.0, vz_des

    def run(self, x, y, z, vx, vy, vz):
        t, x_f, y_f, z_f, vx_f, vy_f, vz_f, h_max = self.get_point_of_contact(x, y, z, vx, vy, vz) # vx_f, vy_f unused
        t, x_des, y_des, z_des, r_des, p_des, yaw_des, vz_des = self.get_heuristics(t, x, y, x_f, y_f, z_f, vz_f, h_max) # garbage yaw