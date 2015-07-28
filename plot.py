import shelve
import os
import numpy as np
import pylab

data = []
for fn in os.listdir('data'):
    if fn.endswith('.db'):
        print fn
        db = shelve.open(os.path.join('data', fn))
        data.extend(db.get('data', []))
        db.close()

pkeys = []
measures = []
for p, r in data:
    for k, v in p.__dict__.items():
        if k not in pkeys:
            pkeys.append(k)
            print k
    for k, v in r.items():
        if k not in measures:
            measures.append(k)
            print k

params = np.zeros((len(data), len(pkeys)), dtype=float)
results = np.zeros((len(data), len(measures)), dtype=float)
for i, d in enumerate(data):
    p, r = d
    for k, v in p.__dict__.items():
        index = pkeys.index(k)
        params[i, index] = v
    for k, v in r.items():
        index = measures.index(k)
        results[i, index] = v

def weight(params, target, smoothing):
    d = params[:,:,None] - target[None,:]
    d = d / smoothing[None, :, None]
    d = np.sum(d ** 2, axis=1)

    return np.exp(-d / 2)

def plot_1d(key, params, results, fixed, smoothing, measure='rmse'):
    index = pkeys.index(key)

    minv = min(params[:,index])
    maxv = max(params[:,index])

    N = 30

    target = np.zeros((len(pkeys), N), dtype=float)
    for k, v in fixed.items():
        target[pkeys.index(k), :] = v

    v = np.linspace(minv, maxv, 30)
    target[index] = v
    print target.shape

    w = weight(params, target, smoothing)



    r = results[:, measures.index(measure)]
    r = np.where(r > 1.0, 1.0, r)

    color = np.max(w, axis=1)

    near_index = color > 0.1

    pylab.figure()
    pylab.scatter(params[near_index,index], r[near_index], c=color[near_index], cmap='gray_r', marker='x')
    pylab.xlabel(key)

    r = np.repeat(r[:, None], N, axis=1)
    a = np.average(r, axis=0, weights=w)

    print a
    print a.shape


smoothing = np.zeros(len(pkeys), dtype=float) + 0.01
plot_1d('sense_noise', params, results, fixed=dict(motor_noise=0.0), 
        smoothing=smoothing)
plot_1d('motor_noise', params, results, fixed=dict(sense_noise=0.0), 
        smoothing=smoothing)
import pylab
pylab.show()    

