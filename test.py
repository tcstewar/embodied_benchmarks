import embodied_benchmarks as bench

D = 2
D_motor = 3
T = 10.0
repetitions = 3
max_freq = 2
dt = 0.001
sys = bench.system.LinearSystem(D, D_motor, scale_add=10, seed=1,
                                motor_delay=0.01, motor_filter=0.01,
                                sensor_delay=0.01, sensor_filter=0.01,
                                scale_inertia=0.3,
                                dt=dt)
ctrl = bench.pid.PID(16, 4, 10, J=sys.J, dt=dt)
desired = bench.signal.Signal(D, T / repetitions, dt=dt, max_freq=max_freq)
eval = bench.evaluate.Evaluate(sys, ctrl, desired)

eval.test(T, plot=True)


