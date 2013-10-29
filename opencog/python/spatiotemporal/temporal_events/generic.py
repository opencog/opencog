from datetime import datetime
from scipy.stats.distributions import rv_frozen
from scipy import integrate
from spatiotemporal.time_intervals import assert_is_time_interval, TimeInterval
from fuzzy.membership_function import MembershipFunction, ProbabilityDistributionPiecewiseLinear
from spatiotemporal.unix_time import UnixTime
from utility.geometric import index_of_first_local_maximum
from utility.numeric.globals import EPSILON

__author__ = 'keyvan'


class TemporalEvent(TimeInterval):
    _distribution_beginning = None
    _distribution_ending = None
    _beginning = None
    _ending = None
    _membership_function = None
    input_list = None

    def __init__(self, a, beginning, ending, b, distribution_beginning, distribution_ending, bins=50):
        assert beginning < ending
        TimeInterval.__init__(self, a, b, bins)
        self._beginning = UnixTime(beginning)
        self._ending = UnixTime(ending)
        self.distribution_beginning = distribution_beginning
        self.distribution_ending = distribution_ending

    @property
    def membership_function(self):
        if self._membership_function is None:
            self._membership_function = MembershipFunction(self)
        return self._membership_function

    def _interval_from_self_if_none(self, a, b, interval):
        if interval is None:
            if (a, b) == (None, None):
                interval = self
            else:
                interval = TimeInterval(a, b)
        else:
            assert_is_time_interval(interval)
        return interval

    def degree_in_interval(self, a=None, b=None, interval=None):
        """
        use either 'a' and 'b' or 'interval'
        """
        interval = self._interval_from_self_if_none(a, b, interval)
        area, error = integrate.quad(self.membership_function, interval.a, interval.b)
        return area / interval.duration

    def to_list(self):
        if self.input_list is None:
            self.input_list = [
                self.a, UnixTime(self.a + EPSILON),
                self.beginning, self.ending,
                UnixTime(self.b - EPSILON), self.b
            ]
        return self.input_list

    def to_dict(self):
        result = {}
        for time_step in self.to_list():
            result[time_step] = self.membership_function(time_step)
        return result

    def plot(self):
        import matplotlib.pyplot as plt
        from spatiotemporal.unix_time import UnixTime

        x_axis = [UnixTime(time).to_datetime() for time in self]
        plt.plot(x_axis, self.membership_function)
        return plt

    @property
    def distribution_beginning(self):
        return self._distribution_beginning

    @distribution_beginning.setter
    def distribution_beginning(self, value):
        assert isinstance(value, rv_frozen)
        self._distribution_beginning = value

    @property
    def distribution_ending(self):
        return self._distribution_ending

    @distribution_ending.setter
    def distribution_ending(self, value):
        assert isinstance(value, rv_frozen)
        self._distribution_ending = value

    @property
    def beginning(self):
        return self._beginning

    @property
    def ending(self):
        return self._ending

    @property
    def _iter_step_beginning(self):
        return float(self.beginning - self.a) / self._bins

    @property
    def _iter_step_ending(self):
        return float(self.b - self.ending) / self._bins

    def __getitem__(self, index):
        if index >= len(self) or index < 0:
            raise IndexError
        if index < self._bins:
            return self._iter_step_beginning * index
        if index in [self._bins, self._bins + 1]:
            return 1
        return self._iter_step_ending * index

    def __len__(self):
        return self._bins * 2

    def __iter__(self):
        return (self[t] for t in xrange(self._bins * 2))

    def __reversed__(self):
        return (self.membership_function(t) for t in reversed(self.to_list()))

    def __repr__(self):
        return '{0}(a: {1}, beginning: {2}, ending:{3}, b:{4})'.format(self.__class__.__name__,
                                                                       self.a, self.beginning, self.ending, self.b)

    def __str__(self):
        return repr(self)


class TemporalEventPiecewiseLinear(TemporalEvent):
    def __init__(self, input_list, output_list, bins=50):
        a, b = input_list[0], input_list[-1]
        index_beginning = index_of_first_local_maximum(output_list)
        index_ending = len(output_list) - index_of_first_local_maximum(reversed(output_list)) - 1

        assert output_list[index_beginning] == 1, "value of 'output_list' in event's 'beginning' should be 1"
        assert output_list[index_ending] == 1, "value of 'output_list' in event's 'ending' should be 1"

        beginning = input_list[index_beginning]
        ending = input_list[index_ending]

        distribution_beginning = ProbabilityDistributionPiecewiseLinear(
            input_list[0:index_beginning + 1],
            output_list[0:index_beginning + 1],
        )
        distribution_ending = ProbabilityDistributionPiecewiseLinear(
            input_list[index_ending:len(input_list)],
            [1 - output_list[t] for t in xrange(index_ending, len(input_list))],
        )

        TemporalEvent.__init__(self, a, beginning, ending, b, distribution_beginning, distribution_ending, bins)
        self.input_list = input_list
        self.output_list = output_list

    def degree_in_interval(self, a=None, b=None, interval=None):
        interval = self._interval_from_self_if_none(a, b, interval)
        return self.membership_function.integrate(interval.a, interval.b) / (interval.b - interval.a)

    def __getitem__(self, index):
        return self.input_list.__getitem__(index)

    def __len__(self):
        return len(self.input_list)

    def __iter__(self):
        return iter(self.input_list)

    def __reversed__(self):
        return reversed(self.input_list)

    def __repr__(self):
        pairs = ['{0}: {1}'.format(self[i], self.output_list[i]) for i in xrange(len(self))]
        return '{0}({1})'.format(self.__class__.__name__, ', '.join(pairs))


class TemporalInstance(TimeInterval):
    def __init__(self, a, b):
        TimeInterval.__init__(self, a, b, 1)

    def plot(self):
        import matplotlib.pyplot as plt
        from spatiotemporal.unix_time import UnixTime

        x_axis = [UnixTime(time).to_datetime() for time in self]
        plt.plot(x_axis, [1, 1])
        return plt


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    event = TemporalInstance(datetime(2010, 1, 1), datetime(2011, 2, 1))
    plt = event.plot()
    plt.show()

    plt.ylim(ymax=1.1)

    event = TemporalEventPiecewiseLinear([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], [0, 0.1, 0.3, 0.7, 1, 1, 0.9, 0.6, 0.1, 0])
    event.plot()
    print event.distribution_beginning.integrate(event.a, event.beginning)
    print event.distribution_beginning.pdf()
    print event.distribution_beginning.cdf()
    print event.distribution_beginning.rvs(10)
    event.distribution_beginning.plot()
    event.distribution_ending.plot().show()
