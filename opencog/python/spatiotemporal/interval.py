from spatiotemporal.time import UnixTime, random_time, is_unix_time, IteratorUnixTime

__author__ = 'keyvan'


def is_interval(item):
    return hasattr(item, 'a') and hasattr(
        item, 'b') and is_unix_time(item.a) and is_unix_time(item.b) and item.a < item.b and hasattr(item, '__iter__')


def assert_is_interval(interval):
    assert is_interval(interval), "passed 'interval' should have 'a' and 'b' attributes, " \
                                  "both of which should be in unix time, and 'b' has to be greater than 'a'"


class Interval(object):
    _iter_step = 1
    _a = None
    _b = None

    def __init__(self, a, b, iter_step=1):
        """
        a and b can be in either datetime or unix time
        """
        self.a = a
        self.b = b
        assert self.a < self.b, "'b' should be greater than 'a'"
        self.iter_step = iter_step

    def random_time(self, start=None, stop=None, probability_distribution=None):
        if start is None:
            start = self.a
        else:
            start = UnixTime(start)
            assert self.a <= start <= self.b, "'start' should be within the interval of the interval"
        if stop is None:
            stop = self.b
        else:
            stop = UnixTime(stop)
            assert self.a <= stop <= self.b, "'stop' should be within the interval of the interval"
        return random_time(start, stop, probability_distribution)

    def x_axis(self):
        axis = []
        for time_step in self:
            axis.append(time_step)
        return axis

    @property
    def a(self):
        return self._a

    @a.setter
    def a(self, value):
        self._a = UnixTime(value)

    @property
    def b(self):
        return self._b

    @b.setter
    def b(self, value):
        self._b = UnixTime(value)

    @property
    def iter_step(self):
        return self._iter_step

    @iter_step.setter
    def iter_step(self, value):
        if value != self._iter_step:
            assert isinstance(value, (int, float, long)), value > 0
            self._iter_step = value

    def __contains__(self, item):
        """
        item can either be unix time or any object with 'a' and 'b' unix time attributes (e.g. a Interval)
        """
        if is_unix_time(item):
            return self.a <= item <= self.b

        return self.a <= item.a and self.b >= item.b

    def __iter__(self):
        return IteratorUnixTime(self.a, self.b, self.iter_step)

    def __len__(self):
        return self.b - self.a

    def __repr__(self):
        return 'spatiotemporal.time.Interval([{0} : {1}])'.format(self.a, self.b)

    def __str__(self):
        return 'from {0} to {1}'.format(self.a, self.b)
