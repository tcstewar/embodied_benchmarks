import embodied_benchmarks as bench

D = 3
D_motor = 5
sys = bench.system.LinearSystem(D, D_motor)
ctrl = bench.pid.PID(10, 0, 0, J=sys.J)
eval = bench.evaluate.Evaluate(sys, ctrl)

eval.test(10, plot=True)


