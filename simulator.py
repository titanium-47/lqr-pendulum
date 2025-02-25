import pygame
import numpy as np
import sys

class PendulumSimulator:
    def __init__(self, controller, m=1.0, l=1.0, g=9.81, scale=250, damping=1.0, hz = 100, decimation = 1, torque_max = 10):
        pygame.init()
        self.torque_max = torque_max
        self.controller = controller
        self.hz = hz
        self.decimation = decimation
        self.m = m
        self.l = l
        self.g = g
        self.scale = scale
        self.damping = damping
        self.theta = 0
        self.theta_dot = 0.0
        self.torque = 0.0
        self.pivot = (400, 150)
        self.width, self.height = 800, 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Pygame Pendulum Simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 36)

        self.input_box = pygame.Rect(10, 130, 140, 32)
        self.color_inactive = pygame.Color('lightskyblue3')
        self.color_active = pygame.Color('dodgerblue2')
        self.color = self.color_inactive
        self.active = False
        self.user_text = ''

    def simulate(self):
        # Simulation loop
        running = True
        counter = 0
        while running:
            # Determine time elapsed (in seconds)
            dt = self.clock.tick(self.hz) / 1000.0  # 60 frames per second

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # If the user clicked on the input_box rect
                    if self.input_box.collidepoint(event.pos):
                        self.active = not self.active
                    else:
                        self.active = False
                    # Change the current color of the input box
                    self.color = self.color_active if self.active else self.color_inactive
                elif event.type == pygame.KEYDOWN:
                    if self.active:
                        if event.key == pygame.K_RETURN:
                            try:
                                # Update the target angle with user input
                                theta_target = np.radians(float(self.user_text))
                                theta_target = (theta_target % (2 * np.pi) + 3 * np.pi) % (2*np.pi) - np.pi
                                self.controller.set_setpoint(theta_target)
                            except ValueError:
                                print("Invalid input. Please enter a number.")
                            self.user_text = ''
                        elif event.key == pygame.K_BACKSPACE:
                            self.user_text = self.user_text[:-1]
                        else:
                            self.user_text += event.unicode

            if counter == 0:
                self.torque = self.controller.control(self.theta, self.theta_dot)[0]
            self.torque = np.clip(self.torque, -self.torque_max, self.torque_max)
            # Calculate angular acceleration:
            #   theta_ddot = - (g/l)*sin(theta) + (torque)/(m*l^2)
            theta_ddot = -(self.g / self.l) * np.sin(self.theta) + (self.torque / (self.m * self.l**2)) - (self.damping / self.m) * self.theta_dot

            # Update the angular velocity and angle (Euler integration)
            self.theta_dot += theta_ddot * dt
            self.theta += self.theta_dot * dt

            # Clear the screen (white background)
            self.screen.fill((255, 255, 255))

            # Calculate the position of the pendulum bob
            x = self.pivot[0] + self.l * self.scale * np.sin(self.theta)
            y = self.pivot[1] + self.l * self.scale * np.cos(self.theta)

            # Draw the rod (line) and the bob (circle)
            pygame.draw.line(self.screen, (0, 0, 0), self.pivot, (x, y), 2)
            pygame.draw.circle(self.screen, (0, 0, 255), (int(x), int(y)), 20)

            self.theta = (self.theta % (2 * np.pi) + 3 * np.pi) % (2*np.pi) - np.pi

            # Convert theta to degrees for display and render the text
            angle_degrees = np.degrees(self.theta)
            angle_text = self.font.render("Theta: {:.2f}°".format(angle_degrees), True, (0, 0, 0))
            self.screen.blit(angle_text, (10, 10))

            # Render and display the applied torque
            torque_text = self.font.render("Torque: {:.2f} N·m".format(self.torque), True, (0, 0, 0))
            self.screen.blit(torque_text, (10, 50))

            target_angle_degrees = np.degrees(self.controller.theta)
            target_text = self.font.render(f"Target Angle: {target_angle_degrees:.2f}°", True, (0, 0, 0))
            self.screen.blit(target_text, (10, 90))

            # Render the input box
            txt_surface = self.font.render(self.user_text, True, self.color)
            width = max(200, txt_surface.get_width() + 10)
            self.input_box.w = width
            self.screen.blit(txt_surface, (self.input_box.x + 5, self.input_box.y + 5))
            pygame.draw.rect(self.screen, self.color, self.input_box, 2)

            # Update the display
            pygame.display.flip()
            counter += 1
            counter %= self.decimation

        # Quit Pygame when the loop is exited
        pygame.quit()
        sys.exit()
