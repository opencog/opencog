from spatiotemporal.unix_time import UnixTime, random_time, is_unix_time

__author__ = 'keyvan'


def is_time_interval(item):
    return hasattr(item, 'a') and hasattr(
        item, 'b') and is_unix_time(item.a) and is_unix_time(item.b) and item.a < item.b


def assert_is_time_interval(interval):
    assert is_time_interval(interval), "passed 'interval' should have 'a' and 'b' attributes, " \
                                       "both of which should be in unix time, and 'b' has to be greater than 'a'"


class TimeInterval(object):
    _a = None
    _b = None
    _iter_step = None

    def __init__(self, a, b, bins=50):
        """
        a and b can be in either datetime or unix time
        """
        assert a < b, "'b' should be greater than 'a'"
        self._a = UnixTime(a)
        self._b = UnixTime(b)
        self.bins = bins

    def random_time(self, start=None, stop=None, probability_distribution=None):
        if start is None:
            start = self.a
        else:
            start = UnixTime(start)
            assert self.a <= start <= self.b, "'start' should be within the interval of 'self'"
        if stop is None:
            stop = self.b
        else:
            stop = UnixTime(stop)
            assert self.a <= stop <= self.b, "'stop' should be within the interval of 'self'"
        return random_time(start, stop, probability_distribution)

    def to_list(self):
        result = []
        for time_step in self:
            result.append(time_step)
        return result

    @property
    def duration(self):
        return float(self.b - self.a)

    @property
    def a(self):
        return self._a

    @property
    def b(self):
        return self._b

    @property
    def bins(self):
        return self._bins

    @bins.setter
    def bins(self, value):
        if value != self._iter_step:
            assert isinstance(value, int), value > 0
            self._bins = value
            self._iter_step = self.duration / value

    def __getitem__(self, index):
        value = self.a + index * self._iter_step
        if value > self.b:
            raise IndexError
        return UnixTime(value)

    def __len__(self):
        return int(self.b - self.a)/int(self._iter_step) + 1

    def __iter__(self):
        return (self[t] for t in xrange(len(self)))

    def __reversed__(self):
        return (self[t] for t in xrange(len(self) - 1, -1, -1))

    def __contains__(self, item):
        """
        item can either be unix time or any object with 'a' and 'b' unix time attributes (e.g. an Interval)
        """
        if is_unix_time(item):
            return self.a <= item and self.b >= item
        return self.a <= item.a and self.b <= item.b

    def __repr__(self):
        return '{0}([{1} : {2}])'.format(self.__class__.__name__, self.a, self.b)

    def __str__(self):
        return 'from {0} to {1}'.format(self.a, self.b)


#class TimeIntervalListBased(list, TimeInterval):
#    def __init__(self, iterable):
#        assert len(iterable) >= 2
#        iterable = sorted(iterable)
#        list_converted_to_unix_time = []
#        first_item = True
#        for current in iterable:
#            current = UnixTime(current)
#            list_converted_to_unix_time.append(current)
#            if first_item:
#                previous = current
#                first_item = False
#                continue
#            assert current > previous
#            previous = current
#        list.__init__(self, list_converted_to_unix_time)
#
#    def to_list(self):
#        return self
#
#    @property
#    def a(self):
#        return self[0]
#
#    @a.setter
#    def a(self, value):
#        assert value > self[1]
#        self[0] = UnixTime(value)
#
#    @property
#    def b(self):
#        return self[-1]
#
#    @b.setter
#    def b(self, value):
#        assert value > self[-2]
#        self[-1] = UnixTime(value)
#
#    def __contains__(self, item):
#        return TimeInterval.__contains__(self, item)
#
#    def __repr__(self):
#        return TimeInterval.__repr__(self)
#
#    def __str__(self):
#        return TimeInterval.__str__(self)


if __name__ == '__main__':
    import time
    a = TimeInterval(1, 100000)
    b = []

    start = time.time()
    ls = a.to_list()

    for t in xrange(len(a)):
        b.append(ls[t])

    list_performance = time.time() - start

    start = time.time()
    for t in xrange(len(a)):
        b.append(a[t])

    print 'time:', list_performance, 'for list vs.', time.time() - start, 'direct'
