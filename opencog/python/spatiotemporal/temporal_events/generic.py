from datetime import datetime
from scipy import integrate
from spatiotemporal.interval import Interval, assert_is_interval
from spatiotemporal.membership_function import FuzzyMembershipFunction

__author__ = 'keyvan'


class BaseTemporalEvent(Interval):

    def __init__(self, a, b, iter_step=1):
        Interval.__init__(self, a, b, iter_step=iter_step)
        self.membership_function = FuzzyMembershipFunction(self, self.membership_function_single_point)

    def membership_function_single_point(self, time_step):
        """
        to override, membership_function calls to it
        alternatively one can directly override membership_function
        """
        return 0

    def _interval_from_self_if_none(self, a, b, interval):
        if interval is None:
            if (a, b) == (None, None):
                interval = self
            else:
                interval = Interval(a, b)
        else:
            assert_is_interval(interval)
        return interval

    def degree_in_interval(self, a=None, b=None, interval=None):
        """
        use either 'a' and 'b' or 'interval'
        """
        interval = self._interval_from_self_if_none(a, b, interval)
        return integrate.quad(self.membership_function, interval.a, interval.b) / interval.duration

    def to_dict(self):
        result = {}
        for time_step in self:
            result[time_step] = self.membership_function(time_step)
        return result

    def __repr__(self):
        return '{0}(a: {1}, b: {2})'.format(self.__class__.__name__, self.a, self.b)

    def __str__(self):
        return repr(self)


class BaseTemporalEventWithCustomListImplementation(BaseTemporalEvent):
    """
    override to_list and use this class
    """
    def __init__(self, a, b):
        BaseTemporalEvent.__init__(self, a, b)

    def to_list(self):
        return [self.a, self.b]

    def __getitem__(self, index):
        return self.to_list().__getitem__(index)

    def __len__(self):
        return len(self.to_list())

    def __iter__(self):
        return iter(self.to_list())


class TemporalEventSimple(BaseTemporalEventWithCustomListImplementation):
    def membership_function_single_point(self, time_step):
        if self.a <= time_step <= self.b:
            return 1
        return 0


class TemporalEventDistributional(BaseTemporalEvent):
    def __init__(self, a, b, pdf, iter_step=1):
        assert callable(pdf), "'pdf' should be callable"
        Interval.__init__(self, a, b, iter_step=iter_step)
        self.membership_function = pdf
        self.membership_function_single_point = pdf


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    event = TemporalEventSimple(datetime(2010, 1, 1), datetime(2011, 2, 1))
    event = plt.plot(event.to_list(), event.membership_function())
    plt.show()