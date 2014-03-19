from numpy import PINF
from math import fabs
from scipy.stats import uniform
from spatiotemporal.temporal_events.trapezium import RelationFormulaTrapezium

__author__ = 'keyvan'

epsilon = 0.0000001
binary_steps = 20

formula = RelationFormulaTrapezium()


def find_dist_on_right(dist_left, dist_right_start_point, same_value):
    dist_left_start_point, dist_left_length = dist_left.args
    return uniform(dist_right_start_point, (dist_left_start_point + dist_left_length
                                            - dist_right_start_point) ** 2 / same_value ** 2 / dist_left_length)


def find_dist_on_right_both(dist_left_1, dist_left_2, same_1, same_2):
    interval = [dist_left_1.args[0], dist_left_1.args[0] + dist_left_1.args[1]]
    alternative_interval = interval
    delta = PINF
    previous_delta = PINF
    for k in xrange(binary_steps):
        dist_right_start_point = (interval[0] + interval[1]) / 2.0
        dist_right = find_dist_on_right(dist_left_1, dist_right_start_point, same_1)
        same_heuristic = formula.compare(dist_left_2, dist_right)[1]
        delta = same_heuristic - same_2
        if fabs(delta) < epsilon:
            return dist_right, delta
        if fabs(delta) < fabs(previous_delta):
            if delta > 0:
                interval[1] = dist_right_start_point
                alternative_interval = [(interval[0] + interval[1]) / 2.0, dist_right_start_point]
            else:
                interval[0] = dist_right_start_point
                alternative_interval = [dist_right_start_point, (interval[0] + interval[1]) / 2.0]
        else:
            interval = alternative_interval
        previous_delta = delta

    return None, delta


def find_config(same_b_beg_a, same_b_beg_c, same_b_end_a, same_b_end_c):
    b_beg_heuristic = uniform(0, 1)
    interval_a = [0.0, 1.0]
    interval_c = [0.0, 1.0]
    for i in xrange(binary_steps):
        delta_1 = PINF
        a_start_point = (interval_a[0] + interval_a[1]) / 2.0
        a_heuristic = find_dist_on_right(b_beg_heuristic, a_start_point, same_b_beg_a)
        for j in xrange(binary_steps):
            c_start_point = (interval_c[0] + interval_c[1]) / 2.0
            c_heuristic = find_dist_on_right(b_beg_heuristic, c_start_point, same_b_beg_c)
            b_end_heuristic, delta_2 = find_dist_on_right_both(a_heuristic, c_heuristic, same_b_end_a, same_b_end_c)
            if b_end_heuristic is not None:
                return a_heuristic, b_beg_heuristic, b_end_heuristic, c_heuristic
            if delta_2 < 0:
                interval_c[1] = c_start_point
            else:
                interval_c[0] = c_start_point
            if fabs(delta_2) < fabs(delta_1):
                delta_1 = delta_2
        if delta_1 > 0:
            interval_a[1] = a_start_point
        else:
            interval_a[0] = a_start_point
    return None


b_beg = uniform(0, 1)
b_end = uniform(4, 4)
a = uniform(0.8, 4.2)
c = uniform(0.4, 6.6)

same_b_beg_a = formula.compare(b_beg, a)[1]
same_b_beg_c = formula.compare(b_beg, c)[1]
same_b_end_a = formula.compare(b_end, a)[1]
same_b_end_c = formula.compare(b_end, c)[1]

print a.args, b_beg.args, b_end.args, c.args
print '-------------'
print formula.compare(b_beg, a)
print formula.compare(b_beg, c)
print formula.compare(b_end, a)
print formula.compare(b_end, c)
print formula.compare(a, c)
print '-------------'
a, b_beg, b_end, c = find_config(same_b_beg_a, same_b_beg_c, same_b_end_a, same_b_end_c)
print a.args, b_beg.args, b_end.args, c.args
print '-------------'
print formula.compare(b_beg, a)
print formula.compare(b_beg, c)
print formula.compare(b_end, a)
print formula.compare(b_end, c)
print formula.compare(a, c)
