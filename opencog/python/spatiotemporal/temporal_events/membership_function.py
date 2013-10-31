from math import fabs
from random import random
from scipy.stats.distributions import rv_frozen
from spatiotemporal.time_intervals import TimeInterval
from spatiotemporal.unix_time import random_time
from utility.generic import convert_dict_to_sorted_lists
from utility.geometric import Function, FunctionPiecewiseLinear,\
    FunctionHorizontalLinear, FunctionComposite, invoke_method_on, FUNCTION_ZERO, FUNCTION_ONE
from utility.numeric.globals import PLUS_INFINITY, MINUS_INFINITY

__author__ = 'keyvan'


class MembershipFunction(Function):
    def __init__(self, temporal_event):
        Function.__init__(self, function_undefined=FUNCTION_ZERO, domain=temporal_event)

    def call_on_single_point(self, time_step):
        a = self.domain.distribution_beginning.cdf(time_step)
        a = self.domain.distribution_ending.cdf(time_step)
        return self.domain.distribution_beginning.cdf(time_step) - self.domain.distribution_ending.cdf(time_step)


class ProbabilityDistributionPiecewiseLinear(TimeInterval, rv_frozen):
    def __init__(self, dictionary_input_output):
        self.input_list, cdf_output_list = convert_dict_to_sorted_lists(dictionary_input_output)
        TimeInterval.__init__(self, self.input_list[0], self.input_list[-1])
        self.cdf = FunctionPiecewiseLinear(dictionary_input_output, function_undefined=FUNCTION_ZERO)
        self.cdf.dictionary_bounds_function[(self.b, PLUS_INFINITY)] = FUNCTION_ONE
        pdf_output_list = []
        dictionary_bounds_function = {}
        for bounds in sorted(self.cdf.dictionary_bounds_function):
            a, b = bounds
            if a in [MINUS_INFINITY, PLUS_INFINITY] or b in [MINUS_INFINITY, PLUS_INFINITY]:
                continue
            pdf_y_intercept = fabs(self.cdf.derivative((a + b) / 2.0))
            pdf_output_list.append(pdf_y_intercept)
            dictionary_bounds_function[bounds] = FunctionHorizontalLinear(pdf_y_intercept)

        self.pdf = FunctionComposite(dictionary_bounds_function,
                                     function_undefined=FUNCTION_ZERO, domain=self.input_list)

        self.roulette_wheel = []
        share = 0
        for bounds in self.pdf.dictionary_bounds_function:
            (a, b) = bounds
            if a in [MINUS_INFINITY, PLUS_INFINITY] and b in [MINUS_INFINITY, PLUS_INFINITY]:
                continue
            share += self.pdf.dictionary_bounds_function[bounds].integrate(a, b)
            self.roulette_wheel.append((a, b, share))

    def interval(self, alpha):
        if alpha == 1:
            return self.a, self.b
        raise NotImplementedError

    def rvs(self, size=None):
        if size is None:
            size = 1
        else:
            assert isinstance(size, int)

        result = []
        start, end = 0, 0
        for i in xrange(size):
            rand = random()
            for a, b, share in self.roulette_wheel:
                if rand < share:
                    start, end = a, b
                    break
            result.append(random_time(start, end))

        if size == 1:
            return result[0]
        return result

    def plot(self):
        import matplotlib.pyplot as plt
        plt.plot(self.to_datetime_list(), self.pdf)
        return plt

    def __getitem__(self, index):
        return self.input_list.__getitem__(index)

    def __len__(self):
        return len(self.input_list)

    def __iter__(self):
        return iter(self.input_list)

    def __reversed__(self):
        return reversed(self.input_list)

    def __str__(self):
        return repr(self)