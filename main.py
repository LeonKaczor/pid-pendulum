import math
import pygame
import matplotlib.pyplot as plt
from config import Config
from physics import CartPendulum, Controller
from optimization import Optimizer

# Colors for pygame simulation
BLACK = (0, 0, 0)
CYAN = (30, 100, 220)
MAGENTA = (220, 50, 50)
DEEP_BLUE = (50, 130, 200)
DARK_GRAY = (180, 180, 180)


class Simulation:
    def __init__(self):
        self.config = Config()
        self.optimizer = Optimizer(self.config)

        # optimilization
        pid_params = self.optimizer.find_pid_parameters()
        energy_params = self.optimizer.find_energy_parameters(pid_params)

        # adding pendulum and controller to the simulation
        self.pendulum = CartPendulum(self.config)
        self.controller = Controller(self.config, energy_params, pid_params)

        # simulation constants
        self.dt = 0.0001
        self.fps = 60
        self.steps_per_frame = int((1 / self.fps) / self.dt)

        # pygame initiation
        pygame.init()
        self.width, self.height = 1300, 750
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.SysFont("Arial", 22)
        self.clock = pygame.time.Clock()
        self.scale = 500

        # saving data for the charts
        self.t = 0
        self.t_list, self.theta_list, self.x_list, self.F_list = [], [], [], []

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # many simulation steps per one frame of simulation
            for _ in range(self.steps_per_frame):
                F, e_x, e_theta = self.controller.get_force(self.pendulum)
                F_real = F = max(-self.config.MAX_FORCE,
                                 min(self.config.MAX_FORCE, F))
                self.pendulum.update(F_real, self.dt)
                self.t += self.dt
                self.t_list.append(self.t)
                self.x_list.append(self.pendulum.x)
                self.theta_list.append(self.pendulum.theta)
                self.F_list.append(F)

            self.render(F, e_x, e_theta)
            self.clock.tick(self.fps)

        pygame.quit()
        self.plot_results()

    def render(self, F: float, e_x: float, e_theta: float):
        self.screen.fill((255, 255, 255))

        # text
        self.screen.blit(self.font.render(
            f"e_x:     {e_x:.3f} m", True, (30, 30, 30)), (20, 20))
        self.screen.blit(self.font.render(
            f"e_theta: {e_theta:.3f} rad", True, (30, 30, 30)), (20, 50))
        self.screen.blit(self.font.render(
            f"F:       {F:.3f} N", True, (30, 30, 30)), (20, 80))

        # cart cordinates
        cart_x = self.width // 2 + self.pendulum.x * self.scale
        cart_y = self.height // 2 + 300

        # cordinated of the pendulum tip
        pend_x = cart_x + self.config.L * \
            self.scale * math.sin(self.pendulum.theta)
        pend_y = cart_y - self.config.L * \
            self.scale * math.cos(self.pendulum.theta)

        # pygame draw
        track_y = cart_y + 20
        pygame.draw.line(self.screen, DARK_GRAY, (0, track_y),
                         (self.width, track_y), 2)

        pygame.draw.line(self.screen, (DEEP_BLUE[0]//2, DEEP_BLUE[1]//2, DEEP_BLUE[2]//2),
                         (cart_x, cart_y), (pend_x, pend_y), 12)
        pygame.draw.line(self.screen, DEEP_BLUE,
                         (cart_x, cart_y), (pend_x, pend_y), 6)

        cart_rect = pygame.Rect(cart_x - 40, cart_y - 20, 80, 40)
        pygame.draw.rect(
            self.screen, (CYAN[0]//2, CYAN[1]//2, CYAN[2]//2), cart_rect, border_radius=10, width=5)
        pygame.draw.rect(self.screen, CYAN, cart_rect, border_radius=10)

        pygame.draw.circle(self.screen, (MAGENTA[0]//2, MAGENTA[1]//2, MAGENTA[2]//2),
                           (int(pend_x), int(pend_y)), 20)
        pygame.draw.circle(self.screen, MAGENTA,
                           (int(pend_x), int(pend_y)), 16)

        pygame.display.flip()

    def plot_results(self):
        plt.subplot(311)
        plt.plot(self.t_list, self.theta_list)
        plt.xlabel("time [s]")
        plt.ylabel("angle to point 0 [rad]")

        plt.subplot(312)
        plt.plot(self.t_list, self.x_list)
        plt.xlabel("time [s]")
        plt.ylabel("cart position [m]")

        plt.subplot(313)
        plt.plot(self.t_list, self.F_list)
        plt.xlabel("time [s]")
        plt.ylabel("Force exerted on the cart [m]")

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    app = Simulation()
    app.run()
