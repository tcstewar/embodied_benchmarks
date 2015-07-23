import numpy as np


class Signal(object):
    def __init__(self, D, L, dt, max_freq, seed=None):
        rng = np.random.RandomState(seed=seed)
        steps = int(max_freq * L)
        self.w = 2 * np.pi * np.arange(steps) / L
        self.A = rng.randn(D, steps) + 1.0j * rng.randn(D, steps)

        power = np.sqrt(np.sum(self.A * self.A.conj()))
        self.A /= power

    def value(self, t):
        s = np.sin(self.w * t) * self.A
        return np.sum(s, axis=1).real
    def dvalue(self, t):
        s = np.cos(self.w * t) * self.w * self.A
        return np.sum(s, axis=1).real



class Environment(object):
    def __init__(self, seed=None):
        self.rng = np.random.RandomState(seed=seed)

class LinearSystem(Environment):
    def __init__(self, d_controlled, d_motor, dt=0.001, seed=None,
            scale_mult=10, scale_add=10, diagonal=False,
            max_sense_noise=0.1, max_motor_noise=0.1,
            period=5.0, max_freq=1.0):
        super(LinearSystem, self).__init__(seed=seed)


        self.d_motor = d_motor
        self.d_controlled = d_controlled
        self.dt = dt

        self.state = self.rng.randn(d_controlled)

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
            x = np.dot(J.T, J)
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

    D_state = 3
    D_motor = 5
    dt = 0.001

    env = LinearSystem(d_controlled=D_state, d_motor=D_motor, diagonal=False, scale_add=5)
    ctrl = PID(100, 10, 1000, J=env.J)
    desired_state = Signal(D_state, L=3.0, dt=dt, max_freq=2.0)

    T = 6.0
    steps = int(T / dt)
    t = np.arange(steps) * dt

    state = np.zeros((D_state, steps), dtype=float)
    desired = np.zeros((D_state, steps), dtype=float)
    sense = np.zeros((D_state, steps), dtype=float)

    m = np.zeros(D_motor, dtype=float)
    for i in range(steps):
        desired[:,i] = desired_state.value(t[i])
        s = env.step(m)
        m = ctrl.step(s, desired[:,i])

        state[:,i] = env.state
        sense[:,i] = s

    import pylab
    pylab.plot(t, state.T, label='state')
    pylab.plot(t, desired.T, label='desired')
    #pylab.plot(sense.T, label='sense')
    #pylab.legend(loc='best')
    pylab.show()





