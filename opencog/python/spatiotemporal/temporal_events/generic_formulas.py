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
    x = [dist_1.pdf(t) for t in range]
    y = [dist_2.pdf(t) for t in reversed(range)]

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


def temporal_relations_between(temporal_event_1, temporal_event_2, prec):
    beginning_a_beginning_b = temporal_event_1.distribution_beginning, temporal_event_2.distribution_ending
    beginning_a_ending_b = temporal_event_1.distribution_beginning, temporal_event_2.distribution_ending
    ending_a_beginning_b = temporal_event_1.distribution_ending, temporal_event_2.distribution_beginning
    ending_a_ending_b = temporal_event_1.distribution_ending, temporal_event_2.distribution_ending

    function_beginning_a_beginning_b = function_convolution_generic(*beginning_a_beginning_b)
    function_beginning_a_ending_b = function_convolution_generic(*beginning_a_ending_b)
    function_ending_a_beginning_b = function_convolution_generic(*ending_a_beginning_b)
    function_ending_a_ending_b = function_convolution_generic(*ending_a_ending_b)

    before = NEGATIVE_INFINITY, -prec
    same = -prec, prec
    after = prec, POSITIVE_INFINITY

    convolutions = {
        beginning_a_beginning_b: (
            function_beginning_a_beginning_b.integral(*before),
            function_beginning_a_beginning_b.integral(*same),
            function_beginning_a_beginning_b.integral(*after)
        ),
        beginning_a_ending_b: (
            function_beginning_a_ending_b.integral(*before),
            function_beginning_a_ending_b.integral(*same),
            function_beginning_a_ending_b.integral(*after)
        ),
        ending_a_beginning_b: (
            function_ending_a_beginning_b.integral(*before),
            function_ending_a_beginning_b.integral(*same),
            function_ending_a_beginning_b.integral(*after)
        ),
        ending_a_ending_b: (
            function_ending_a_ending_b.integral(*before),
            function_ending_a_ending_b.integral(*same),
            function_ending_a_ending_b.integral(*after)
        ),
    }

    before = 0
    same = 1
    after = 2

    result = {
        'p':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][before] * convolutions[ending_a_ending_b][before],

        'm':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][same] * convolutions[ending_a_ending_b][before],

        'o':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][before],

        'F':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][same],

        'D':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],

        's':
        convolutions[beginning_a_beginning_b][same] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][before],

        'e':
        convolutions[beginning_a_beginning_b][same] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][same],

        'd':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][before],

        'f':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][same],

        'O':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],

        'M':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][same] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],

        'P':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][after] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],
    }

    summed_up = sum(result.values())
    for predicate in result:
        result[predicate] /= summed_up
    return result


def empirical_convolve(event_1, event_2, prec=0.25, size=50000):
    import matplotlib.pyplot as plt

    times_overlapped = float(0)
    times_preceded = float(0)
    times_met = float(0)
    size = 50000
    x = []


    for i in xrange(size):
        rv1 = beginning_a.rvs()
        if rv1 <= 0:
            rv1 = 0.01
        rv2 = ending_a.rvs()
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
    print 'before:', p
    print 'same:', m
    print 'after:', o

    #plt.ylim(ymax=0.15)
    plt.hist(x, 50, normed=True)


if __name__ == '__main__':
    from scipy.stats import norm, uniform, expon
    from numpy import NINF as NEGATIVE_INFINITY, PINF as POSITIVE_INFINITY

    #beginning_a, ending_a = expon(loc=1, scale=4.5), norm(loc=30, scale=2)
    #beginning_b, ending_b = expon(loc=25, scale=4.5), norm(loc=60, scale=2)

    #beginning_a, ending_a = uniform(loc=1, scale=4), uniform(loc=6, scale=4)
    #beginning_b, ending_b = uniform(loc=0, scale=11), uniform(loc=13, scale=4)

    beginning_a, ending_a = uniform(loc=4, scale=1), uniform(loc=10, scale=2)
    beginning_b, ending_b = uniform(loc=2, scale=4), uniform(loc=7, scale=7)

    event_1 = TemporalEvent(beginning_a, ending_a)
    event_2 = TemporalEvent(beginning_b, ending_b)

    relations = temporal_relations_between(event_1, event_2, 0.25)
    for predicate in relations:
        print predicate, ':', relations[predicate]
    print sum(relations.values())

    event_1.plot()
    event_2.plot().show()
