from spatiotemporal.unix_time import UnixTime

__author__ = 'keyvan'


class FuzzyMembershipFunction(object):
    def __init__(self, interval, calculate_for_single_point):
        assert callable(calculate_for_single_point), "'calculate_for_single_point' should be callable"
        self.a = interval.a
        self.b = interval.b
        self.interval = interval
        self.calculate_for_single_point = calculate_for_single_point

    @property
    def iter_step(self):
        return self.interval.iter_step

    def __call__(self, x=None):
        if x is None:
            x = self.interval

        result = []
        try:
            for point in x:
                point = UnixTime(point)
                result.append(self.calculate_for_single_point(point))
        except:
            return self.calculate_for_single_point(x)
        return result

    def __iter__(self):
        return (self[t] for t in xrange(len(self)))

    def __getitem__(self, index):
        return self.calculate_for_single_point(self.interval[index])

    def __len__(self):
        return len(self.interval)
