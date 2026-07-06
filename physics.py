import math
from config import Config


class CartPendulum:

    def __init__(self, config):
        self.cfg = config
        self.reset(self.cfg.delta)

    def reset(self, initial_theta_deg):
        self.x = 0.0
        self.d1x = 0.0
        self.theta = math.radians(initial_theta_deg)
        self.d1theta = 0.0

    def update(self, force, dt):
        # getting force
        F = max(-self.cfg.MAX_FORCE, min(self.cfg.MAX_FORCE, force))

        sin_t = math.sin(self.theta)
        cos_t = math.cos(self.theta)
        M_m = self.cfg.M + self.cfg.m
        m_L = self.cfg.m * self.cfg.L

        # equations of motion
        d2theta = (self.cfg.g * sin_t + cos_t * ((-F - m_L * (self.d1theta**2) * sin_t) / M_m)) / \
                  (self.cfg.L * (1 - (self.cfg.m * (cos_t**2)) / M_m))

        d2x = (F + m_L * (self.d1theta**2) *
               sin_t - m_L * d2theta * cos_t) / M_m

        # Euler integration
        self.d1theta += d2theta * dt
        self.d1x += d2x * dt
        self.theta += self.d1theta * dt
        self.x += self.d1x * dt


class Controller:

    def __init__(self, config, energy_params, pid_params, pid_bar_deg=15.0):
        self.cfg = config
        self.k_e, self.Kp_x_energy, self.kd_x_energy = energy_params
        self.Kp_theta, self.Kd_theta, self.Kp_x, self.Kd_x = pid_params

        self.pid_activation_bar = math.radians(pid_bar_deg)
        self.E_ref = self.cfg.m * self.cfg.g * self.cfg.L

    def get_force(self, pendulum, x_needed=0.0, theta_needed=0.0):
        # theta norm changes angle to 0 - 2pi -> this enables full rotation during swing-up phase
        theta_norm = (pendulum.theta + math.pi) % (2 * math.pi) - math.pi
        e_x = pendulum.x - x_needed

        # if the pendulum is close to equilibrium then we switch from nergy swing-up to pid
        if abs(theta_norm) < self.pid_activation_bar:
            e_theta = theta_norm - theta_needed
            F_theta = (self.Kp_theta * e_theta) + \
                (self.Kd_theta * pendulum.d1theta)
            F_x = (self.Kp_x * e_x) + (self.Kd_x * pendulum.d1x)
            F = F_theta + F_x
            return F, e_x, e_theta

        # Swing-up
        else:
            E = 0.5 * self.cfg.m * self.cfg.L**2 * pendulum.d1theta**2 + \
                self.cfg.m * self.cfg.g * self.cfg.L * math.cos(theta_norm)

            F_swing = self.k_e * (E - self.E_ref) * \
                pendulum.d1theta * math.cos(theta_norm)
            F_drift = -self.Kp_x_energy * e_x - self.kd_x_energy * pendulum.d1x
            F = F_swing + F_drift
            return F, e_x, theta_norm
