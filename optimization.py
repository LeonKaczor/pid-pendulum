import math
from scipy.optimize import minimize
from config import Config
from physics import CartPendulum


class Optimizer:

    def __init__(self, config: Config):
        self.config = config
        self.dt = 0.01

    def find_pid_parameters(self):
        initial_guess = [110, 10, 5, 3]
        bounds = ((0, 500), (0, 100), (0, 100), (0, 50))

        def cost_function(params):
            Kp_theta, Kd_theta, Kp_x, Kd_x = params
            pendulum = CartPendulum(self.config)
            pendulum.reset(15.0)
            t = 0
            total_cost = 0

            while t < 5:
                theta_norm = (pendulum.theta +
                              math.pi) % (2 * math.pi) - math.pi

                F_theta = (Kp_theta * theta_norm) + \
                    (Kd_theta * pendulum.d1theta)
                F_x = (Kp_x * pendulum.x) + (Kd_x * pendulum.d1x)

                F = F_theta + F_x
                F = max(-self.config.MAX_FORCE, min(self.config.MAX_FORCE, F))

                pendulum.update(F, self.dt)

                cost_x = 1000 * max(0, abs(pendulum.x) -
                                    self.config.rail_length)**2
                cost_drift = 1 * (pendulum.x**2)
                cost_theta = 100 * (theta_norm**2)
                cost_force = 0.001 * (F**2)

                total_cost += cost_theta + cost_x + cost_drift + cost_force
                t += self.dt

            return total_cost

        print("Optimizing the PID parameters...")
        res = minimize(cost_function, initial_guess, bounds=bounds)
        return res.x[0], res.x[1], res.x[2], res.x[3]

    def find_energy_parameters(self, pid_params):
        initial_guess = [8, 8, 2]
        bounds = ((0, 500), (0, 300), (0, 100))

        Kp_theta, Kd_theta, Kp_x, Kd_x = pid_params
        pid_activation_bar = math.radians(15.0)

        def cost_function(params):
            k_e, Kp_x_energy, kd_x_energy = params
            pendulum = CartPendulum(self.config)
            pendulum.reset(self.config.delta)
            t = 0
            total_cost = 0

            while t < 5:
                theta_norm = (pendulum.theta +
                              math.pi) % (2 * math.pi) - math.pi

                if abs(theta_norm) < pid_activation_bar:
                    F_theta = (Kp_theta * theta_norm) + \
                        (Kd_theta * pendulum.d1theta)
                    F_x = (Kp_x * pendulum.x) + (Kd_x * pendulum.d1x)
                    F = F_theta + F_x
                else:
                    E = 0.5 * self.config.m * self.config.L**2 * pendulum.d1theta**2 + \
                        self.config.m * self.config.g * \
                        self.config.L * math.cos(theta_norm)

                    F_swing = k_e * (E - self.config.m * self.config.g *
                                     self.config.L) * pendulum.d1theta * math.cos(theta_norm)
                    F_drift = -Kp_x_energy * pendulum.x - kd_x_energy * pendulum.d1x
                    F = F_swing + F_drift

                # Restricting the force to the give boundry
                F = max(-self.config.MAX_FORCE, min(self.config.MAX_FORCE, F))

                pendulum.update(F, self.dt)

                cost_x = 1000 * max(0, abs(pendulum.x) -
                                    self.config.rail_length)**2
                cost_drift = 1 * (pendulum.x**2)
                cost_theta = 100 * (theta_norm**2)
                cost_force = 0.001 * (F**2)

                total_cost += cost_theta + cost_x + cost_drift + cost_force
                t += self.dt

            return total_cost

        print("Optimizing the energy parameters...")
        res = minimize(cost_function, initial_guess, bounds=bounds)
        return res.x[0], res.x[1], res.x[2]
