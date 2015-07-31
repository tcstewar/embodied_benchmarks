import embodied_benchmarks as bench

D = 1
D_motor = D
T = 20.0
repetitions = 5
max_freq = 2
dt = 0.001
sys = bench.system.System(D, D_motor, scale_add=5, seed=10,
                                motor_delay=0.01, motor_filter=0.01,
                                sensor_delay=0.01, sensor_filter=0.01,
                                scale_inertia=0.1,
                                scale_mult=1,
                                sense_noise=0.1, motor_noise=0.1,
                                nonlinear=True,
                                motor_scale=10,
                                diagonal=True,
                                dt=dt)
#ctrl = bench.pid.PID(4, 1, 1, J=None, dt=dt)
ctrl = bench.pid.PDAdapt(dim=D_motor, Kp=4, Kd=1, dt=dt, synapse=0.005,
                         learning_rate=0.1, n_neurons=500)
desired = bench.signal.Signal(D, T / repetitions, dt=dt, max_freq=max_freq)
eval = bench.evaluate.Evaluate(sys, ctrl, desired)


print eval.test(T, plot=True, eval_time= T - T / repetitions)


