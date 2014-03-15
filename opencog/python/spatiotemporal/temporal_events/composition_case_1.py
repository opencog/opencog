from math import fabs
from scipy.stats import uniform
from spatiotemporal.temporal_events import RelationFormulaGeometricMean
from spatiotemporal.temporal_events.trapezium import RelationFormulaTrapezium

__author__ = 'keyvan'

b_beg = uniform(10, 3)
b_end = uniform(1001, 3)
c_actual = uniform(11, 991)

f = RelationFormulaTrapezium()
same_beg = f.compare(b_beg, c_actual)[1]
same_end = f.compare(b_end, c_actual)[1]

interval = [10, 13.0]
x = []

for i in xrange(40):
    c_start_point = (interval[0] + interval[1]) / 2
    length = ((13.0 - c_start_point) / same_beg) ** 2 / 3.0
    c_heuristic = uniform(c_start_point, length)
    same_heuristic = f.compare(b_end, c_heuristic)[1]
    same_heuristic_beg = f.compare(b_beg, c_heuristic)[1]
    delta = same_end - same_heuristic
    x.append(delta)
    if fabs(delta) < 0.000000001:
        print same_beg
        print same_heuristic_beg
        print same_end
        print same_heuristic
        print c_heuristic.args
        break
    if delta > 0:
        interval[1] = c_start_point
    else:
        interval[0] = c_start_point

import pylab as plt
plt.plot(x)
plt.show()
