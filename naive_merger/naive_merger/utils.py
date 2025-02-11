
import numpy as np

class cc:
    H = '\033[95m'
    OK = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    W = '\033[93m'
    F = '\033[91m'
    E = '\033[0m'
    B = '\033[1m'
    U = '\033[4m'

def is_zeros(v):
    return np.allclose(v, np.zeros(len(v)))

def entropy(v):
    return -np.sum([x * np.log2(x) for x in v])

def safe_entropy(v):
    v = np.asarray(v, dtype=float)
    v += np.finfo(float).eps  # add epsilon against x / 0
    v = v / np.sum(v) # normalize
    return entropy(v)

def normalized_entropy(v):
    if len(v) == 1: return v[0]
    return safe_entropy(v) / np.log2(len(v))

def cross_entropy(v, q):
    return -np.sum([vx * np.log2(qx) for vx, qx in zip(v, q)])

def safe_cross_entropy(v, q):
    assert len(v) == len(q)
    v, q = np.asarray(v, dtype=float), np.asarray(q, dtype=float)
    v += np.finfo(float).eps  # add epsilon against x / 0
    q += np.finfo(float).eps  # add epsilon against x / 0
    v = v / np.sum(v) # normalize
    q = q / np.sum(q) # normalize
    return cross_entropy(v, q)

def normalized_cross_entropy(v, q):
    return safe_cross_entropy(v, q) / np.log2(len(v))

def  diagonal_cross_entropy(v):
    if len(v) == 1: return v[0]
    return [normalized_cross_entropy(np.eye(len(v))[i], v) for i in range(len(v))]
