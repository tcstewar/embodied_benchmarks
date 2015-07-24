import numpy as np

from . import signal

class Evaluate(object):
    def __init__(self, system, controller, desired):
        self.system = system
        self.controller = controller
        self.desired = desired

    def test(self, T, plot=False):
        self.system.reset()
        self.controller.reset()

        dt = self.system.dt
        D = self.system.d_state

        steps = int(T / dt)
        m = np.zeros(self.system.d_motor, dtype=float)

        if plot:
            state = np.zeros((steps, D), dtype=float)
            desired = np.zeros((steps, D), dtype=float)
            sense = np.zeros((steps, D), dtype=float)
            motor = np.zeros((steps, self.system.d_motor), dtype=float)

        for i in range(steps):
            t = i * dt
            d = self.desired.value(t)
            s = self.system.step(m)
            m = self.controller.step(s, d)

            if plot:
                state[i] = self.system.state
                desired[i] = d
                sense[i] = s
                motor[i] = m

        if plot:
            import pylab
            timesteps = np.arange(steps) * dt
            pylab.plot(timesteps, state, label='state')
            pylab.plot(timesteps, desired, label='desired')
            pylab.show()
