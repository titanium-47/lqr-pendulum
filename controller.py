from scipy.linalg import solve_discrete_are
import numpy as np

class LQR:
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.P = solve_discrete_are(A, B, Q, R)
        self.K = np.linalg.inv(R + B.T @ self.P @ B) @ B.T @ self.P @ A

    def reset(self, A):
        self.A = A
        self.P = solve_discrete_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R + self.B.T @ self.P @ self.B) @ self.B.T @ self.P @ self.A
    
    def control(self, x):
        return -self.K @ x

class PendulumLQR(LQR):
    def __init__(self, m, l, theta, Q, R, dt = 0.01, g = 9.8):
        self.dt = dt
        self.theta = theta
        self.u_init = m*g*l*np.sin(theta)
        self.mgl = m*g*l
        A = np.array([
            [1, dt],
            [-g/l*np.cos(theta)*dt, 1]
        ])
        B = np.array([
            [0],
            [dt/(m*l**2)]
        ])
        super().__init__(A, B, Q, R)

    def set_setpoint(self, theta):
        A = np.array([
            [1, self.dt],
            [-self.mgl * np.sin(theta), 1]
        ])
        self.theta = theta
        self.u_init = self.mgl*np.sin(theta)
        return super().reset(A)
    
    def control(self, theta, theta_dot):
        x = np.array([theta-self.theta, theta_dot])
        return super().control(x) + self.u_init