import embodied_benchmarks as bench

D = 1
D_motor = 1
T = 10.0
repetitions = 3
max_freq = 2
dt = 0.001
sys = bench.system.System(D, D_motor, scale_add=2, seed=3,
                                motor_delay=0.01, motor_filter=0.01,
                                sensor_delay=0.01, sensor_filter=0.01,
                                scale_inertia=0.1,
                                sense_noise=0.1, motor_noise=0.1,
                                nonlinear=True,
                                diagonal=True,
                                dt=dt)
ctrl = bench.pid.PID(0.4, 0.2, 0.01, J=None, dt=dt)
desired = bench.signal.Signal(D, T / repetitions, dt=dt, max_freq=max_freq)
eval = bench.evaluate.Evaluate(sys, ctrl, desired)


print eval.test(T, plot=True, eval_time= T - T / repetitions)


