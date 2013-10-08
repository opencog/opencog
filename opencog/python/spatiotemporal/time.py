from calendar import timegm
from datetime import datetime
from random import random
from scipy.stats import t
from scipy.stats.distributions import rv_frozen

__author__ = 'keyvan'

TEMPORAL_RELATIONS = {
    'p': 'precedes',
    'm': 'meets',
    'o': 'overlaps',
    'F': 'finished by',
    'D': 'contains',
    's': 'starts',
    'e': 'equals',
    'S': 'started by',
    'd': 'during',
    'f': 'finishes',
    'O': 'overlapped by',
    'M': 'met by',
    'P': 'preceded by'
}


def is_unix_time(time):
    return isinstance(time, (int, float, long)) and time > 0


def convert_to_unix_time(time):
    if isinstance(time, datetime):
        return timegm(time.utctimetuple())
    assert is_unix_time(time), 'passed parameter is not a valid unix time'
    return time


def random_time(start, stop, probability_distribution=None):
    """
    start and stop can be in either datetime or unix time
    return value is in unix time
    """
    start = convert_to_unix_time(start)
    stop = convert_to_unix_time(stop)
    assert start < stop, "'stop' should be greater than start"
    if probability_distribution is None:
        return random() * (stop - start) + start
    assert isinstance(probability_distribution,
                      rv_frozen), "'probability_distribution' should be a scipy frozen distribution"
    random_value = probability_distribution.rvs()
    if random_value < start:
        return start
    if random_value > stop:
        return stop
    return random_value


class Period(object):
    iter_step = 1

    def __init__(self, a, b):
        """
        a and b can be in either datetime or unix time
        """
        self.a = convert_to_unix_time(a)
        self.b = convert_to_unix_time(b)
        assert self.a < self.b, "'b' should be greater than 'a'"

    def random_time(self, start=None, stop=None, probability_distribution=None):
        if start is None:
            start = self.a
        else:
            start = convert_to_unix_time(start)
            assert self.a <= start <= self.b, "'start' should be within the interval of the period"
        if stop is None:
            stop = self.b
        else:
            stop = convert_to_unix_time(stop)
            assert self.a <= stop <= self.b, "'stop' should be within the interval of the period"
        return random_time(start, stop, probability_distribution)

    @property
    def integer_range(self):
        int_a = int(self.a)
        if not isinstance(self.a, (int, long)):
            int_a += 1
        if isinstance(self.b, (int, long)):
            int_b = self.b + 1
        else:
            int_b = int(self.b) + 1
        return xrange(int_a, int_b, self.iter_step)

    def __contains__(self, item):
        return self.a <= item.a and self.b >= item.b

    def __iter__(self):
        return iter(self.integer_range)

    def __len__(self):
        return float(self.b - self.a)

    def __repr__(self):
        return 'spatiotemporal.time.Period([{0} : {1}])'.format(self.a, self.b)

    def __str__(self):
        return 'from {0} to {1}'.format(
            datetime.fromtimestamp(int(self.a)).strftime('%Y-%m-%d %H:%M:%S'),
            datetime.fromtimestamp(int(self.b)).strftime('%Y-%m-%d %H:%M:%S'))


class BaseTemporalEvent(object):
    def __init__(self, a, b):
        self.a = convert_to_unix_time(a)
        self.b = convert_to_unix_time(b)
        self.period = Period(a, b)

    def _pdf_single_point(self, time_step):
        """
        to override, default pdf calls to it
        alternatively one can directly override pdf
        """
        pass

    def pdf(self, x):
        """
        time can either be a single value or a collection
        """
        result = []
        try:
            for time_step in x:
                result.append(self._pdf_single_point(time_step))
        except:
            return self._pdf_single_point(x)
        return result

    def _period_pre_check(self, period):
        if period is None:
            period = self.period
        else:
            assert isinstance(period, Period), "'period' should be of type 'spatiotemporal.time.Period'"
        return period

    def probability_in_period(self, period=None):
        #TODO replace by CalculateCenterOfMass
        period = self._period_pre_check(period)
        return max(self.pdf(period))

    @property
    def iter_step(self):
        return self.period.iter_step

    @iter_step.setter
    def iter_step(self, value):
        self.period.iter_step = value

    def __iter__(self):
        return iter(self.period)

    def __len__(self):
        return len(self.period)

    def __repr__(self):
        return 'spatiotemporal.time.TemporalEvent(a:{0}, b:{1})'.format(self.period.a, self.period.b)

    def __str__(self):
        return repr(self)


class TemporalEvent(BaseTemporalEvent):
    def __init__(self, a, b, pdf):
        BaseTemporalEvent.__init__(self, a, b)
        assert callable(pdf), "'pdf' should be callable"
        self.pdf = pdf
        self._pdf_single_point = pdf


def _interpolation_a_to_beginning(t, a, beginning):
    _a = 1 / (beginning - a)
    _b = -1 * _a * a
    return _a * t + _b


def _interpolation_ending_to_b(t, ending, b):
    _a = -1 / (b - ending)
    _b = -1 * _a * b
    return _a * t + _b


def _interpolation_a_to_b(t, a, beginning, ending, b):
    t = convert_to_unix_time(t)
    if t <= a or t >= b:
        return 0
    if a < t < beginning:
        return _interpolation_a_to_beginning(t, a, beginning)
    if ending < t < b:
        return _interpolation_ending_to_b(t, ending, b)
    return 1


class PiecewiseTemporalEvent(BaseTemporalEvent):
    beginning_factor = 5
    ending_factor = 5

    def __init__(self, a, b, beginning=None, ending=None, beginning_factor=None, ending_factor=None):
        """
        start and end can be in either datetime or unix time
        """
        if (beginning, ending) != (None, None):
            assert (beginning_factor, ending_factor) == (None, None), "PiecewiseTemporalEvent init only accepts " \
                                                                      "either 'beginning_factor' and 'ending_factor' " \
                                                                      "or 'beginning' and 'ending'"
        BaseTemporalEvent.__init__(self, a, b)

        if beginning_factor is not None:
            assert beginning_factor > 0
            self.beginning_factor = beginning_factor
        if ending_factor is not None:
            assert ending_factor > 0
            self.ending_factor = ending_factor

        if (beginning, ending) != (None, None):
            assert self.period.a < beginning < ending < self.period.b, "'beginning' and 'ending' should be " \
                                                                       "within the interval of event's period and " \
                                                                       "'ending' should be greater than 'beginning'"
            self.beginning, self.ending = beginning, ending
        else:
            self.beginning, self.ending = 0, 0
            while not self.period.a < self.beginning < self.ending < self.period.b:
                self.beginning = self.period.random_time(
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        self.period.a + len(self) / self.beginning_factor,
                        len(self) / self.beginning_factor
                    )
                )

                self.ending = self.period.random_time(
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        self.period.b - len(self) / self.ending_factor,
                        len(self) / self.ending_factor
                    )
                )

    def _pdf_single_point(self, time_step):
        return _interpolation_a_to_b(time_step, self.a, self.beginning, self.ending, self.b)

    def probability_in_period(self, period=None):
        period = self._period_pre_check(period)
        if self.a < period.a < self.beginning and self.ending < period.b < self.b:
            return 1
        return max(self._pdf_single_point(period.a), self._pdf_single_point(period.b))

    def __repr__(self):
        return 'spatiotemporal.time.PiecewiseTemporalEvent(a:{0} , beginning:{1}, ending:{2}, b:{3})'.format(
            self.period.a, self.beginning, self.ending, self.period.b)


class TemporalRelation(list):
    def __init__(self, constituent_string, temporal_event_1, temporal_event_2):
        assert isinstance(
            temporal_event_1, BaseTemporalEvent
        ) and isinstance(
            temporal_event_2, BaseTemporalEvent
        ), "'temporal_event_1' and 'temporal_event_2' should be of type 'BaseTemporalEvent' or of a subclass thereof"
        list.__init__(self)
        self.append(constituent_string)
        self.temporal_event_1 = temporal_event_1
        self.temporal_event_2 = temporal_event_2

    def append(self, item):
        for char in item:
            assert isinstance(
                char, str
            ) and len(char) is 1 and char in TEMPORAL_RELATIONS, "'item' should be an iterable of characters"
            list.append(self, char)

    def degree(self):
        return 0.5

    def __eq__(self, other):
        assert isinstance(other, (TemporalRelation, str)), "'other' should be of type 'TemporalRelation' or 'str'"
        if isinstance(other, TemporalRelation):
            if (other.temporal_event_1, other.temporal_event_2) != (self.temporal_event_1, self.temporal_event_2):
                return False
        for char in other:
            if char not in self:
                return False
        return True

    def __str__(self):
        if len(self) == len(TEMPORAL_RELATIONS):
            return 'TemporalRelation(pmoFDseSdfOMP(full))'
        if self == 'oFDseSdfO':
            return 'TemporalRelation(oFDseSdfO(concur))'
        return 'TemporalRelation({0}({1}))'.format(
            ''.join(self),
            ', '.join([TEMPORAL_RELATIONS[char] for char in self]))

    def __repr__(self):
        return str(self)


def generate_random_events(size=20):
    events = []
    year_2010 = Period(datetime(2010, 1, 1), datetime(2011, 1, 1))
    iter_step = 100

    for i in xrange(size):
        start = year_2010.random_time()
        end = year_2010.random_time(start)
        event = PiecewiseTemporalEvent(start, end)
        event.iter_step = iter_step
        events.append(event)

    return events


def create_event_relation_hashtable(temporal_events):
    table = {}

    for A in temporal_events:
        for B in temporal_events:
            if B == A:
                continue
            for C in temporal_events:
                if C in (A, B):
                    continue
                for r1 in TEMPORAL_RELATIONS:
                    for r2 in TEMPORAL_RELATIONS:
                        for r3 in TEMPORAL_RELATIONS:
                            relation1 = TemporalRelation(r1, A, B)
                            relation2 = TemporalRelation(r2, B, C)
                            relation3 = TemporalRelation(r3, A, C)
                            degrees = (relation1.degree(), relation2.degree(), relation3.degree())
                            if degrees != (0, 0, 0):
                                table[(r1, r2, r3)] = degrees

    return table


if __name__ == '__main__':
    events = generate_random_events()
    import matplotlib.pyplot as plt

    for event in events:
        y = event.pdf(event)
        plt.plot(event.period.integer_range, y)

    plt.show()

    #events = generate_random_events(10)
    #table = create_event_relation_hashtable(events)
    #print table
