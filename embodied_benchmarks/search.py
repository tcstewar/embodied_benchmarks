import embodied_benchmarks as bench
import numpy as np
import shelve
import time

class Params:
    def __str__(self):
        r = []
        for k, v in sorted(self.__dict__.items()):
            r.append('%s=%r' % (k, v))
        return ', '.join(r)

def sample(rng):
    p = Params()
    p.sense_noise = rng.uniform(0, 0.1)
    p.motor_noise = rng.uniform(0, 0.1)
    return p

def test(p):
    D = 2
    D_motor = 3
    T = 20.0
    repetitions = 3
    max_freq = 1
    dt = 0.001
    sys = bench.system.LinearSystem(D, D_motor, seed=1, scale_add=10,
                                    motor_delay=0.01, motor_filter=0.01,
                                    sensor_delay=0.01, sensor_filter=0.01,
                                    scale_inertia=0.1,
                                    sense_noise=p.sense_noise, 
                                    motor_noise=p.motor_noise,
                                    dt=dt)
    ctrl = bench.pid.PID(10, 4, 10, J=sys.J, dt=dt)
    desired = bench.signal.Signal(D, T / repetitions, dt=dt, max_freq=max_freq)
    eval = bench.evaluate.Evaluate(sys, ctrl, desired)

    return eval.test(T, plot=False, eval_time= T - T / repetitions)

def collect_data(N, filename=None):
    if filename is None:
        filename = time.strftime('data/%Y%m%d-%H%M%S.db')
    rng = np.random.RandomState()
    db = shelve.open(filename)
    data = db.get('data', [])
    for i in range(N):
        p = sample(rng)
        result = test(p)
        print i, p, result
        data.append((p, result))
    db['data'] = data
    db.close()
