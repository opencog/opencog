from spatiotemporal.time_intervals import TimeInterval
from spatiotemporal.unix_time import random_time
from math import fabs
from scipy.integrate import quad
from scipy.stats import uniform
import matplotlib.pyplot as plt

__author__ = 'keyvan'

times_overlapped = float(0)
times_preceded = float(0)
times_met = float(0)
size = 100000
prec = 2
x = []
y = []

v = uniform(loc=-10, scale=13)

for i in xrange(size):
    instance_1 = TimeInterval(0, random_time(1, 10))
    instance_2 = TimeInterval(random_time(7, 11), 12)
    d = instance_1.b - instance_2.a

    if fabs(d) <= prec:
        times_met += 1
    elif instance_1.b < instance_2.a:
        times_preceded += 1
    else:
        times_overlapped += 1
    x.append(d)
    #r = v.rvs()
    #print r
    #print v.pdf(r)

    #e = (1 - fabs(r)) * v.pdf(r)
    #print e
    #y.append(e)

o = times_overlapped / size
p = times_preceded / size
m = times_met / size
print 'o:', o
print 'p:', p
print 'm:', m

plt.hist(x, 50, normed=True)
plt.show()