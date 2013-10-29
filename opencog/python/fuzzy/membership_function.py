from datetime import datetime
from math import fabs
from scipy.stats.distributions import rv_frozen
from spatiotemporal.unix_time import UnixTime
from utility.geometric import Function, FunctionPiecewiseLinear, FunctionHorizontalLinear, FunctionComposite

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
        self.cdf = FunctionPiecewiseLinear(input_list, output_list, FunctionHorizontalLinear(0))
        bounds_function_dictionary = {}
        for bounds in self.cdf.functions:
            a, b = bounds
            bounds_function_dictionary[bounds] = FunctionHorizontalLinear(
                fabs(self.cdf.derivative((a + b) / 2.0)))

        FunctionComposite.__init__(self, bounds_function_dictionary)

    def pdf(self, x):
        if x is None:
            x = self.cdf.input_list

        return invoke_function_on(self, x)

    def cdf(self, x):
        if x is None:
            x = self.cdf.input_list

        return invoke_function_on(self.cdf, x)