from datetime import datetime
from math import fabs
from random import random
from scipy.stats.distributions import rv_frozen
from spatiotemporal.unix_time import UnixTime, random_time
from utility.geometric import Function, FunctionPiecewiseLinear, FunctionHorizontalLinear, FunctionComposite
from utility.numeric.globals import PLUS_INFINITY, MINUS_INFINITY

__author__ = 'keyvan'


def invoke_function_on(function, sequence_or_point):
    result = []
    try:
        for point in sequence_or_point:
            if type(point) is datetime:
                point = UnixTime(point)
            result.append(function(point))
    except:
        if type(sequence_or_point) is datetime:
            sequence_or_point = UnixTime(sequence_or_point)
        return function(sequence_or_point)
    return result


class MembershipFunction(Function):
    def __init__(self, temporal_event):
        self.temporal_event = temporal_event

    def invoke_on_point(self, time_step):
        if self.temporal_event.a >= time_step or self.temporal_event.b <= time_step:
            return 0
        if self.temporal_event.beginning <= time_step <= self.temporal_event.ending:
            return 1
        if self.temporal_event.beginning > time_step:
            return self.temporal_event.distribution_beginning.cdf(time_step)
        return 1 - self.temporal_event.distribution_ending.cdf(time_step)

    def __call__(self, time=None):
        if time is None:
            time = self.temporal_event.to_list()

        return invoke_function_on(self.invoke_on_point, time)

    def __getitem__(self, index):
        return self.invoke_on_point(self.temporal_event.to_list()[index])

    def __len__(self):
        return len(self.temporal_event.to_list())

    def __iter__(self):
        return (self[t] for t in xrange(len(self)))


class ProbabilityDistributionPiecewiseLinear(FunctionComposite, rv_frozen):
    function_undefined = FunctionHorizontalLinear(0)

    def __init__(self, input_list, output_list):
        self.cumulative_density_function = FunctionPiecewiseLinear(input_list, output_list, FunctionHorizontalLinear(0))
        bounds_function_dictionary = {}
        for bounds in self.cumulative_density_function.functions:
            a, b = bounds
            if a in [MINUS_INFINITY, PLUS_INFINITY] or b in [MINUS_INFINITY, PLUS_INFINITY]:
                continue
            bounds_function_dictionary[bounds] = FunctionHorizontalLinear(
                fabs(self.cumulative_density_function.derivative((a + b) / 2.0)))

        FunctionComposite.__init__(self, bounds_function_dictionary)

    def pdf(self, x=None):
        if x is None:
            x = self.cumulative_density_function.input_list

        return invoke_function_on(self, x)

    def cdf(self, x=None):
        if x is None:
            x = self.cumulative_density_function.input_list

        return invoke_function_on(self.cumulative_density_function, x)

    def rvs(self, size=None):
        if size is None:
            size = 1
        else:
            assert isinstance(size, int)

        roulette_wheel = []
        share = 0
        for bounds in self.functions:
            (a, b) = bounds
            if a in [MINUS_INFINITY, PLUS_INFINITY] and b in [MINUS_INFINITY, PLUS_INFINITY]:
                continue
            share += self.functions[bounds].integrate(a, b)
            roulette_wheel.append((a, b, share))

        result = []
        start, end = 0, 0
        for i in xrange(size):
            rand = random()
            for a, b, share in roulette_wheel:
                if rand < share:
                    start, end = a, b
                    break
            result.append(random_time(start, end))

        if size == 1:
            return result[0]
        return result

    def plot(self):
        import matplotlib.pyplot as plt
        from spatiotemporal.unix_time import UnixTime

        x = []
        y = []
        for bounds in sorted(self.functions):
            (a, b) = bounds
            if a not in [MINUS_INFINITY, PLUS_INFINITY] and b not in [MINUS_INFINITY, PLUS_INFINITY]:
                y.append(self.functions[bounds](a))
                y.append(self.functions[bounds](b))
                a, b = UnixTime(a).to_datetime(), UnixTime(b).to_datetime()
                x.append(a)
                x.append(b)
        plt.plot(x, y)
        return plt