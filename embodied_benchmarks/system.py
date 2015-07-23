import numpy as np

class LinearSystem(object):
    def __init__(self, d_state, d_motor, dt=0.001, seed=None,
            scale_mult=10, scale_add=10, diagonal=False,
            sense_noise=0.1, motor_noise=0.1,
            period=5.0, max_freq=1.0):

        self.rng = np.random.RandomState(seed=seed)

        self.d_motor = d_motor
        self.d_state = d_state
        self.dt = dt


        if diagonal:
            assert d_state == d_motor
            self.J = np.abs(np.diag(self.rng.randn(d_motor))) * scale_mult
        else:
            self.J = self.rng.randn(d_motor, d_state) * scale_mult
        self.sense_noise = sense_noise
        self.motor_noise = motor_noise

        self.additive = self.rng.randn(d_state) * scale_add

        self.reset()
    def reset(self):
        self.state = self.rng.randn(self.d_state)


    def step(self, motor):
        motor = motor + self.rng.randn(self.d_motor) * self.motor_noise
        dstate = (np.dot(motor, self.J) + self.additive) * self.dt
        self.state = self.state + dstate
        return self.state + self.rng.randn(self.d_state) * self.sense_noise
