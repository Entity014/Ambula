import math
def clip(x, lo, hi): return max(lo, min(hi, x))
def wrap_pi(a): 
    pi = math.pi
    return (a + pi) % (2*pi) - pi
def sgn(x): return 1.0 if x >= 0.0 else -1.0
