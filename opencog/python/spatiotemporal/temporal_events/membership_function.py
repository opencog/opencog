from math import fabs
from random import random
from scipy.stats.distributions import rv_frozen
from spatiotemporal.time_intervals import TimeInterval
from spatiotemporal.unix_time import random_time, UnixTime
from utility.generic import convert_dict_to_sorted_lists
from utility.functions import Function, FunctionPiecewiseLinear,\
    FunctionHorizontalLinear, FunctionComposite, FUNCTION_ZERO, FUNCTION_ONE, FunctionLinear
from numpy import PINF as POSITIVE_INFINITY, NINF as NEGATIVE_INFINITY
from utility.numeric.globals import EPSILON

__author__ = 'keyvan'


class MembershipFunction(Function):
    def __init__(self, temporal_event):
        Function.__init__(self, function_undefined=FUNCTION_ZERO, domain=temporal_event)

    def call_on_single_point(self, time_step):
        return self.domain.distribution_beginning.cdf(time_step) - self.domain.distribution_ending.cdf(time_step)


class ProbabilityDistributionPiecewiseLinear(list, TimeInterval, rv_frozen):
    dist = 'ProbabilityDistributionPiecewiseLinear'
    _mean = None
    asd = None

    def __init__(self, dictionary_input_output):
        cdf_input_list, cdf_output_list = convert_dict_to_sorted_lists(dictionary_input_output)
        list.__init__(self, cdf_input_list)
        TimeInterval.__init__(self, self[0], self[-1], 2)
        self.cdf = FunctionPiecewiseLinear(dictionary_input_output, function_undefined=FUNCTION_ZERO)
        self.cdf.dictionary_bounds_function[(self.b, POSITIVE_INFINITY)] = FUNCTION_ONE
        pdf_output_list = []
        dictionary_bounds_function = {}
        for bounds in sorted(self.cdf.dictionary_bounds_function):
            a, b = bounds
            if a in [NEGATIVE_INFINITY, POSITIVE_INFINITY] or b in [NEGATIVE_INFINITY, POSITIVE_INFINITY]:
                continue
            pdf_y_intercept = fabs(self.cdf.derivative((a + b) / 2.0))
            pdf_output_list.append(pdf_y_intercept)
            dictionary_bounds_function[bounds] = FunctionHorizontalLinear(pdf_y_intercept)

        self.pdf = FunctionComposite(dictionary_bounds_function, function_undefined=FUNCTION_ZERO, domain=self,
                                     is_normalised=True)

        self.roulette_wheel = []

        self._mean = 0
        for bounds in sorted(self.pdf.dictionary_bounds_function):
            (a, b) = bounds
            if a in [NEGATIVE_INFINITY, POSITIVE_INFINITY] and b in [NEGATIVE_INFINITY, POSITIVE_INFINITY]:
                continue
            cdf = self.cdf.dictionary_bounds_function[bounds]
            pdf = self.pdf.dictionary_bounds_function[bounds]
            share = cdf(b)
            self.roulette_wheel.append((a, b, share))

            self._mean += (a + b) / 2.0 * pdf(a) * (b - a)

    def std(self):
        # Not properly implemented
        return 0

    def stats(self, moments='mv'):
        # Not properly implemented
        # m, s, k
        return self.mean(), 0, 0

    def mean(self):
        return self._mean

    def interval(self, alpha):
        if alpha == 1:
            return self.a, self.b
        raise NotImplementedError("'interval' is not implemented for 'alpha' other than 1")

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

    # def plot(self):
    #     import matplotlib.pyplot as plt
    #     x_axis, y_axis = [], []
    #     for time_step in self:
    #         x_axis.append(UnixTime(time_step - EPSILON).to_datetime())
    #         x_axis.append(UnixTime(time_step + EPSILON).to_datetime())
    #         y_axis.append(self.pdf(time_step - EPSILON))
    #         y_axis.append(self.pdf(time_step + EPSILON))
    #     plt.plot(x_axis, y_axis)
    #     return plt

    def plot(self):
        import matplotlib.pyplot as plt
        x_axis, y_axis = [], []
        for time_step in self:
            x_axis.append(time_step - EPSILON)
            x_axis.append(time_step + EPSILON)
            y_axis.append(self.pdf(time_step - EPSILON))
            y_axis.append(self.pdf(time_step + EPSILON))
        plt.plot(x_axis, y_axis)
        return plt

    def __hash__(self):
        return object.__hash__(self)

    def __repr__(self):
        return TimeInterval.__repr__(self)

    def __str__(self):
        return repr(self)
