from math import fabs
from numpy import linspace, convolve, array
from spatiotemporal.temporal_events import TemporalEvent
from spatiotemporal.temporal_events.util import calculate_bounds_of_probability_distribution
from spatiotemporal.temporal_interval_handling import calculateCenterMass
from utility.geometric import FunctionPiecewiseLinear, FunctionHorizontalLinear

__author__ = 'keyvan'


def function_convolution_generic(dist_1, dist_2, bins=50):
    a1, b1 = calculate_bounds_of_probability_distribution(dist_1)
    a2, b2 = calculate_bounds_of_probability_distribution(dist_2)

    convolution_bounds_a, convolution_bounds_b = min(a1, a2), max(b1, b2)

    delta = fabs(convolution_bounds_a - convolution_bounds_b) / bins
    range = linspace(convolution_bounds_a, convolution_bounds_b, bins)
    x = array([dist_1.pdf(t) for t in range])
    y = array([dist_2.pdf(t) for t in range])

    c = convolve(x, y)
    dictionary_convolution = {}
    for t in xrange(len(c)):
        dictionary_convolution[delta * t] = c[t]
    bias = calculateCenterMass(dictionary_convolution)[0] + dist_2.mean() - dist_1.mean()
    dictionary_convolution_biased = {}
    for t in dictionary_convolution:
        dictionary_convolution_biased[t - bias] = dictionary_convolution[t]

    convolution_function = FunctionPiecewiseLinear(dictionary_convolution_biased, FunctionHorizontalLinear(0))
    return convolution_function.normalised()


def emperical_convolve(dist_1, dist_2, prec=0.25, size=50000):
    import matplotlib.pyplot as plt

    times_overlapped = float(0)
    times_preceded = float(0)
    times_met = float(0)
    size = 50000
    x = []
    event_1 = TemporalEvent(uniform(loc=-100, scale=1), dist_1)
    event_2 = TemporalEvent(dist_2, uniform(loc=300, scale=10))

    for i in xrange(size):
        rv1 = dist_1.rvs()
        if rv1 <= 0:
            rv1 = 0.01
        rv2 = dist_2.rvs()
        if rv2 >= 12:
            rv2 = 11.99
        instance_1 = event_1.instance()
        instance_2 = event_2.instance()
        d = instance_1.b - instance_2.a

        if fabs(d) <= prec:
            times_met += 1
        elif instance_1.b < instance_2.a:
            times_preceded += 1
        else:
            times_overlapped += 1
        x.append(d)

    o = times_overlapped / size
    p = times_preceded / size
    m = times_met / size
    print 'p:', p
    print 'm:', m
    print 'o:', o

    plt.ylim(ymax=0.15)
    plt.hist(x, 50, normed=True)


if __name__ == '__main__':
    from scipy.stats import norm, uniform, expon
    from numpy import NINF as NEGATIVE_INFINITY, PINF as POSITIVE_INFINITY

    for dist_1, dist_2 in [
            (expon(loc=1, scale=4.5), norm(loc=9, scale=2)),
            (uniform(loc=1, scale=9), uniform(loc=7, scale=4))]:

        bounds_1 = calculate_bounds_of_probability_distribution(dist_1)
        bounds_2 = calculate_bounds_of_probability_distribution(dist_2)

        prec = 0.25

        fn = function_convolution_generic(dist_1, dist_2, 150)
        plt = fn.plot()

        emperical_convolve(dist_1, dist_2, prec)

        print 'precede:', fn.integral(NEGATIVE_INFINITY, -prec)
        print 'meet:', fn.integral(-prec, prec)
        print 'overlap:', fn.integral(prec, POSITIVE_INFINITY)
        print '-------------------------------------\n'
        plt.show()