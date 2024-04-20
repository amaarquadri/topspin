import yaml
import math

class Config:
    def __init__(self):
        self.config = None
        self.get_config()

        self.g = None
        self.e = None

        self.x_des = None
        self.y_des = None
        self.z_des = None
        self.z_paddle = None

        self.roll_k = None
        self.roll_max = None
        self.pitch_k = None
        self.pitch_max = None
    
    def get_config(self):
        with open('config.yml', 'r') as file:
            self.config = yaml.safe_load(file)

        self.g = self.config["constants"]["g"]
        self.e = self.config["constants"]["e"]
        
        self.x_des = self.config["params"]["x_des"]
        self.y_des = self.config["params"]["y_des"]
        self.z_des = self.config["params"]["z_des"]
        self.z_paddle = self.config["params"]["z_paddle"]

        self.roll_k = self.config["params"]["roll_k"]
        self.roll_max = self.config["params"]["roll_max"]
        self.pitch_k = self.config["params"]["pitch_k"]
        self.pitch_max = self.config["params"]["pitch_max"]