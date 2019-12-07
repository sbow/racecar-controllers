# lininterp.py
# from https://stackoverflow.com/questions/50508262/using-look-up-tables-in-python/56629804#56629804?newreg=b9ef2a2e2109439bbf56405cd84bafd2

from bisect import bisect_left

def lookup(x, xs, ys):
    if x <= xs[0]:  return ys[0]
    if x >= xs[-1]: return ys[-1]

    i = bisect_left(xs, x)
    k = (x - xs[i-1])/(xs[i] - xs[i-1])
    y = k*(ys[i]-ys[i-1]) + ys[i-1]

    return y
