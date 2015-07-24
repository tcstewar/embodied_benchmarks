import numpy as np

class PID(object):
    def __init__(self, Kp, Kd=0, Ki=0, J=None, tau_d=0.1, dt=0.001):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        if J is not None:
            x = np.dot(J.T, J)
            scale = np.linalg.det(x) ** (1.0 / x.shape[0])
            self.JT = J.T / scale
        else:
            self.JT = None

        self.scale = np.exp(-dt / tau_d)
        self.dt = dt
        self.reset()

    def reset(self):
        self.prev_state = None
        self.dstate = None
        self.istate = None

    def step(self, state, desired_state, desired_dstate=0):
        if self.prev_state is None:
            self.prev_state = state
            self.dstate = np.zeros_like(state)
            self.istate = np.zeros_like(state)
        else:
            d = state - self.prev_state
            self.dstate = self.dstate * self.scale + d * (1.0 - self.scale)
            self.istate += self.dt * (desired_state - state)


        v = (self.Kp * (desired_state - state) +
             self.Kd * (desired_dstate - self.dstate) +
             self.Ki * self.istate)
        if self.JT is not None:
            v = np.dot(v, self.JT)
        return v
