import numpy as np


class Environment(object):
    def __init__(self, seed=None):
        self.rng = np.random.RandomState(seed=seed)

class LinearSystem(Environment):
    def __init__(self, d_controlled, d_motor, dt=0.001, seed=None,
            scale_mult=10, scale_add=10, diagonal=False,
            max_sense_noise=0.1, max_motor_noise=0.1):
        super(LinearSystem, self).__init__(seed=seed)

        self.d_motor = d_motor
        self.d_controlled = d_controlled
        self.dt = dt

        self.state = self.rng.randn(d_controlled)
        self.desired = self.rng.randn(d_controlled)

        if diagonal:
            assert d_controlled == d_motor
            self.J = np.abs(np.diag(self.rng.randn(d_motor))) * scale_mult
        else:
            self.J = self.rng.randn(d_motor, d_controlled) * scale_mult
        self.sense_noise = self.rng.uniform(0, max_sense_noise)
        self.motor_noise = self.rng.uniform(0, max_motor_noise)

        self.additive = self.rng.rand(d_controlled) * scale_add

    def step(self, motor):
        motor = motor + self.rng.randn(self.d_motor) * self.motor_noise
        dstate = (np.dot(motor, self.J) + self.additive) * self.dt
        self.state = self.state + dstate
        return self.state + self.rng.randn(self.d_controlled) * self.sense_noise

class Controller(object):
    pass

class PID(Controller):
    def __init__(self, Kp, Kd=0, Ki=0, J=None, tau_d=0.1, dt=0.001):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        if J is not None:
            x = np.dot(J, J.T)
            scale = np.linalg.det(x) ** (1.0 / x.shape[0])
            self.JT = J.T / scale
        else:
            self.JT = None

        self.prev_state = None
        self.dstate = None
        self.istate = None
        self.scale = np.exp(-dt / tau_d)
        self.dt = dt
    def step(self, state, desired_state):
        if self.prev_state is None:
            self.prev_state = None
            self.dstate = np.zeros_like(state)
            self.istate = np.zeros_like(state)
        else:
            d = state - self.prev_state
            self.dstate = self.dstate * self.scale + d * (1.0 - self.scale)
            self.istate += self.dt * (desired_state - state)


        v = (self.Kp * (desired_state - state) +
             self.Kd * (-self.dstate) +
             self.Ki * self.istate)
        if self.JT is not None:
            v = np.dot(v, self.JT)
        return v


if __name__ == '__main__':
    env = LinearSystem(d_controlled=3, d_motor=3, diagonal=True, scale_add=10)
    ctrl = PID(100, 10, 1000, J=env.J)

    state = []
    desired = []
    sense = []

    m = np.zeros(env.d_motor, dtype=float)
    for i in range(1000):
        s = env.step(m)
        #ds = env.desired - s

        #m = np.dot(ds, env.J.T) * 10
        m = ctrl.step(s, env.desired)

        state.append(env.state)
        desired.append(env.desired)
        sense.append(s)

    import pylab
    pylab.plot(state, label='state')
    pylab.plot(desired, label='desired')
    pylab.plot(sense, label='sense')
    pylab.legend(loc='best')
    pylab.show()





