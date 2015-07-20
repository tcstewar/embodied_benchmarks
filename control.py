import numpy as np


class Environment(object):
    def __init__(self, seed=None):
        self.rng = np.random.RandomState(seed=seed)

class Control(Environment):
    def __init__(self, d_controlled, d_motor, dt=0.001, seed=None):
        super(Control, self).__init__(seed=seed)

        self.d_motor = d_motor
        self.d_controlled = d_controlled
        self.dt = dt

        self.state = self.rng.randn(d_controlled)
        self.desired = self.rng.randn(d_controlled)

        self.J = self.rng.rand(d_motor, d_controlled)
        self.sense_noise = self.rng.uniform(0, 0.1)
        self.motor_noise = self.rng.uniform(0, 0.1)

        self.additive = self.rng.rand(d_controlled) * 10

    def step(self, motor):
        motor = motor + self.rng.randn(self.d_motor) * self.motor_noise
        dstate = (np.dot(motor, self.J) + self.additive) * self.dt
        self.state = self.state + dstate
        return self.state + self.rng.randn(self.d_controlled) * self.sense_noise


if __name__ == '__main__':
    env = Control(d_controlled=2, d_motor=3)

    state = []
    desired = []
    sense = []

    m = np.zeros(env.d_motor, dtype=float)
    for i in range(1000):
        s = env.step(m)
        ds = env.desired - s

        m = np.dot(ds, env.J.T) * 10

        state.append(env.state)
        desired.append(env.desired)
        sense.append(s)

    import pylab
    pylab.plot(state, label='state')
    pylab.plot(desired, label='desired')
    pylab.plot(sense, label='sense')
    pylab.legend(loc='best')
    pylab.show()
    




