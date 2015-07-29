import numpy as np

from . import signal

class Evaluate(object):
    def __init__(self, system, controller, desired):
        self.system = system
        self.controller = controller
        self.desired = desired

    def test(self, T, plot=False, eval_time=None):
        if eval_time is None:
            eval_time = T

        self.system.reset()
        self.controller.reset()

        dt = self.system.dt
        D = self.system.d_state

        steps = int(T / dt)
        m = np.zeros(self.system.d_motor, dtype=float)

        eval_steps = int(eval_time / dt)

        if plot:
            state = np.zeros((steps, D), dtype=float)
            desired = np.zeros((steps, D), dtype=float)
            sense = np.zeros((steps, D), dtype=float)
            motor = np.zeros((steps, self.system.d_motor), dtype=float)

        for i in range(steps):
            t = i * dt
            d = self.desired.value(t)
            dd = self.desired.dvalue(t)
            s = self.system.step(m)
            m = self.controller.step(s, d, desired_dstate=dd*0)

            if plot:
                state[i] = self.system.state
                desired[i] = d
                sense[i] = s
                motor[i] = m

        s = sense[-eval_steps:,0]
        d = desired[-eval_steps:,0]
        corr = np.correlate(s, d, 'full')
        index = np.argmax(corr)
        delay = index - eval_steps
        diff = desired[-eval_steps - delay: -delay] - sense[-eval_steps:]

        rmse = np.sqrt(np.mean(diff.flatten()**2))

        if plot:
            import pylab
            timesteps = np.arange(steps) * dt
            pylab.subplot(3,1,1)
            pylab.plot(timesteps, desired, label='desired')
            pylab.plot(timesteps, state, label='state')
            pylab.subplot(3,1,2)
            pylab.plot(timesteps[:-delay], desired[:-delay], label='desired')
            pylab.plot(timesteps[:-delay], state[delay:], label='state')
            pylab.subplot(3,1,3)
            pylab.plot(timesteps, motor, label='motor')

            #pylab.figure()
            #pylab.plot(np.correlate(s, d, 'full'))
            #pylab.plot(timesteps[-eval_steps:], 
            #           diff)
            #pylab.figure()
            #pylab.hist(diff.flatten(), 50)

            pylab.show()

        return dict(rmse=rmse, delay=dt * delay)
