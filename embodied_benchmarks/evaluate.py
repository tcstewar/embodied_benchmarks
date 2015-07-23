import numpy as np

from . import signal

class Evaluate(object):
    def __init__(self, system, controller):
        self.system = system
        self.controller = controller

    def test(self, T, max_freq=2.0, repetitions=2, plot=False):
        self.system.reset()
        self.controller.reset()

        dt = self.system.dt
        D = self.system.d_state
        desired_state = signal.Signal(D, L=T / repetitions, dt=dt, 
                                      max_freq=max_freq)

        steps = int(T / dt)
        m = np.zeros(self.system.d_motor, dtype=float)

        if plot:
            state = np.zeros((steps, D), dtype=float)
            desired = np.zeros((steps, D), dtype=float)
            sense = np.zeros((steps, D), dtype=float)
            motor = np.zeros((steps, self.system.d_motor), dtype=float)

        for i in range(steps):
            t = i * dt
            d = desired_state.value(t)
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




if __name__ == '__main__':
    from . import system
    from . import pid

    D = 3
    D_motor = 5
    sys = system.LinearSystem(D, D_motor)
    ctrl = pid.PID(10, 0, 0, J=sys.J)

    eval = Evaluate(sys, ctrl)

    eval.text(10, plot=True)


