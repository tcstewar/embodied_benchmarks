import numpy as np

class LinearSystem(object):
    def __init__(self, d_state, d_motor, dt=0.001, seed=None,
            scale_mult=10, scale_add=10, diagonal=False,
            sense_noise=0.1, motor_noise=0.1,
            motor_delay=0, motor_filter=None,
            sensor_delay=0, sensor_filter=None):

        self.rng = np.random.RandomState(seed=seed)

        self.d_motor = d_motor
        self.d_state = d_state
        self.dt = dt

        sensor_steps = int(sensor_delay / dt) + 1
        self.sensor_delay = np.zeros((sensor_steps, d_state), dtype=float)
        motor_steps = int(motor_delay / dt) + 1
        self.motor_delay = np.zeros((motor_steps, d_motor), dtype=float)
        self.sensor_index = 0
        self.motor_index = 0

        self.sensor = np.zeros(d_state, dtype=float)
        self.motor = np.zeros(d_motor, dtype=float)
        if sensor_filter is None or sensor_filter < dt:
            self.sensor_filter_scale = 0.0
        else:
            self.sensor_filter_scale = np.exp(-dt / sensor_filter)
        if motor_filter is None or motor_filter < dt:
            self.motor_filter_scale = 0.0
        else:
            self.motor_filter_scale = np.exp(-dt / motor_filter)

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
        self.sensor_delay *= 0
        self.motor_delay *= 0


    def step(self, motor):
        self.motor_delay[self.motor_index] = motor
        self.motor_index = (self.motor_index + 1) % len(self.motor_delay)
        motor = self.motor_delay[self.motor_index]

        motor = motor + self.rng.randn(self.d_motor) * self.motor_noise
        self.motor = (self.motor * self.motor_filter_scale +
                      motor * (1.0 - self.motor_filter_scale))
        dstate = (np.dot(self.motor, self.J) + self.additive) * self.dt
        self.state = self.state + dstate

        sensor = self.state + self.rng.randn(self.d_state) * self.sense_noise
        self.sensor = (self.sensor * self.sensor_filter_scale +
                       sensor * (1.0 - self.sensor_filter_scale))

        self.sensor_delay[self.sensor_index] = self.sensor
        self.sensor_index = (self.sensor_index + 1) % len(self.sensor_delay)
        return self.sensor_delay[self.sensor_index]