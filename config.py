from dataclasses import dataclass


@dataclass
class Config:
    g = 9.80665  # gravitational acceleration
    MAX_FORCE = 20.0  # maximum force on the cart
    M = 1  # mass of the cart
    m = 0.5  # mass of the pendulum
    L = 0.8  # lenght of the pendulum
    delta = 16.0  # initial angle in degrees (0 degrees = standing vertically)
    # lenght from the middle of the platform to the maximum position of the cart
    rail_length = 1.0
