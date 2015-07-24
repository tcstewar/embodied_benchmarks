import embodied_benchmarks as bench
import numpy as np

class Params:
    pass

def sample(rng):
    p = Params()
    p.sense_noise = rng.uniform(0, 1.0)
    p.motor_noise = rng.uniform(0, 1.0)
    return p


def test(p):
    D = 2
    D_motor = 3
    T = 10.0
    repetitions = 3
    max_freq = 2
    dt = 0.001
    sys = bench.system.LinearSystem(D, D_motor, scale_add=10,
                                    motor_delay=0.01, motor_filter=0.01,
                                    sensor_delay=0.01, sensor_filter=0.01,
                                    sense_noise=p.sense_noise, 
                                    motor_noise=p.motor_noise,
                                    dt=dt)
    ctrl = bench.pid.PID(10, 4, 10, J=sys.J, dt=dt)
    desired = bench.signal.Signal(D, T / repetitions, dt=dt, max_freq=max_freq)
    eval = bench.evaluate.Evaluate(sys, ctrl, desired)

    return eval.test(T, plot=False, eval_time= T - T / repetitions)

p = sample(np.random.RandomState(seed=1))
print p.__dict__
print test(p)


