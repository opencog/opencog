from scipy.stats.distributions import rv_frozen
from spatiotemporal.temporal_events.relation_formulas import FormulaCreator, RelationFormulaGeometricMean, BaseRelationFormula, RelationFormulaConvolution
from spatiotemporal.temporal_events.util import calculate_bounds_of_probability_distribution
from spatiotemporal.time_intervals import check_is_time_interval, TimeInterval
from spatiotemporal.temporal_events.membership_function import MembershipFunction, ProbabilityDistributionPiecewiseLinear
from spatiotemporal.unix_time import UnixTime
from utility.generic import convert_dict_to_sorted_lists
from utility.geometric import FunctionPiecewiseLinear, FUNCTION_ZERO

__author__ = 'keyvan'


class TemporalEvent(list, TimeInterval):
    _distribution_beginning = None
    _distribution_ending = None
    _beginning = None
    _ending = None
    _dict = None

    def __init__(self, distribution_beginning, distribution_ending,
                 bins=50, relation_formula=None):
        if not isinstance(distribution_beginning, rv_frozen):
            raise TypeError("'distribution_beginning' should be a scipy frozen distribution")
        if not isinstance(distribution_ending, rv_frozen):
            raise TypeError("'distribution_ending' should be a scipy frozen distribution")
        self._distribution_beginning = distribution_beginning
        self._distribution_ending = distribution_ending

        a, beginning = calculate_bounds_of_probability_distribution(distribution_beginning)
        ending, b = calculate_bounds_of_probability_distribution(distribution_ending)

        self._beginning = UnixTime(beginning)
        self._ending = UnixTime(ending)
        self.membership_function = MembershipFunction(self)
        bins_beginning = bins / 2
        bins_ending = bins - bins_beginning
        self.interval_beginning = TimeInterval(a, beginning, bins_beginning)
        self.interval_ending = TimeInterval(ending, b, bins_ending)
        list.__init__(self, self.interval_beginning + self.interval_ending)
        TimeInterval.__init__(self, a, b, bins)
        # if relation_formula is None:
        #     relation_formula = RelationFormulaGeometricMean()
        # elif not isinstance(relation_formula, BaseRelationFormula):
        #     raise TypeError("'relation_formula' should be of type 'BaseRelationFormula'")
        relation_formula = RelationFormulaGeometricMean()
        relation_formula.bounds[distribution_beginning] = self.a, self.beginning
        relation_formula.bounds[distribution_ending] = self.ending, self.b
        self._formula_creator = FormulaCreator(relation_formula)

    def degree(self, time_step=None, a=None, b=None, interval=None):
        """
        usage: provide 'time_step' or 'a' and 'b' or 'interval'
        """
        if time_step is not None:
            return self.membership_function(time_step)

        if interval is None:
            if (a, b) == (None, None):
                interval = self
            else:
                interval = TimeInterval(a, b)
        else:
            check_is_time_interval(interval)
        return integral(self.membership_function, interval.a, interval.b)

    def temporal_relations_with(self, other):
        return self._formula_creator.temporal_relations_between(self, other)

    def instance(self):
        return TemporalInstance(self.distribution_beginning.rvs(), self.distribution_ending.rvs())

    def to_dict(self):
        if self._dict is None:
            self._dict = {}
            for time_step in self.to_list():
                self._dict[time_step] = self.membership_function(time_step)
        return self._dict

    # def plot(self, show_distributions=False):
    #     import matplotlib.pyplot as plt
    #     plt.plot(self.to_datetime_list(), self.membership_function())
    #     if show_distributions:
    #         if hasattr(self.distribution_beginning, 'plot'):
    #             self.distribution_beginning.plot()
    #         else:
    #             plt.plot(self.interval_beginning.to_datetime_list(),
    #                      self.distribution_beginning.pdf(self.interval_beginning))
    #         if hasattr(self.distribution_ending, 'plot'):
    #             self.distribution_ending.plot()
    #         else:
    #             plt.plot(self.interval_ending.to_datetime_list(),
    #                      self.distribution_ending.pdf(self.interval_ending))
    #     return plt

    def plot(self, plt=None, show_distributions=False):
        if plt is None:
            import matplotlib.pyplot as plt
        plt.plot(self.to_float_list(), self.membership_function())
        if show_distributions:
            if hasattr(self.distribution_beginning, 'plot'):
                self.distribution_beginning.plot()
            else:
                plt.plot(self.interval_beginning.to_float_list(),
                         self.distribution_beginning.pdf(self.interval_beginning))
            if hasattr(self.distribution_ending, 'plot'):
                self.distribution_ending.plot()
            else:
                plt.plot(self.interval_ending.to_float_list(),
                         self.distribution_ending.pdf(self.interval_ending))
        return plt


    @property
    def distribution_beginning(self):
        return self._distribution_beginning

    @property
    def distribution_ending(self):
        return self._distribution_ending

    @property
    def beginning(self):
        return self._beginning

    @property
    def ending(self):
        return self._ending

    def __mul__(self, other):
        return self.temporal_relations_with(other)

    def __str__(self):
        return repr(self)


class TemporalEventPiecewiseLinear(TemporalEvent):
    def __init__(self, dictionary_beginning, dictionary_ending, bins=50):
        input_list_beginning, output_list_beginning = convert_dict_to_sorted_lists(dictionary_beginning)
        for i in xrange(1, len(input_list_beginning)):
            if not dictionary_beginning[input_list_beginning[i]] > dictionary_beginning[input_list_beginning[i - 1]]:
                raise TypeError("values of 'dictionary_beginning' should be increasing in time")

        input_list_ending, output_list_ending = convert_dict_to_sorted_lists(dictionary_ending)
        for i in xrange(1, len(input_list_ending)):
            if not dictionary_ending[input_list_ending[i]] < dictionary_ending[input_list_ending[i - 1]]:
                raise TypeError("values of 'dictionary_ending' should be decreasing in time")
        dictionary_ending = {}
        for i, time_step in enumerate(input_list_ending):
            dictionary_ending[time_step] = output_list_ending[len(input_list_ending) - i - 1]
        input_list_ending, output_list_ending = convert_dict_to_sorted_lists(dictionary_ending)

        distribution_beginning = ProbabilityDistributionPiecewiseLinear(dictionary_beginning)
        distribution_ending = ProbabilityDistributionPiecewiseLinear(dictionary_ending)

        TemporalEvent.__init__(self, distribution_beginning, distribution_ending, bins=bins)
        self._list = sorted(set(input_list_beginning + input_list_ending))
        self.membership_function = FunctionPiecewiseLinear(self.to_dict(), FUNCTION_ZERO)

    def __getitem__(self, index):
        return self._list.__getitem__(index)

    def __len__(self):
        return len(self._list)

    def __iter__(self):
        return iter(self._list)

    def __reversed__(self):
        return reversed(self._list)

    def __repr__(self):
        pairs = ['{0}: {1}'.format(self[i], self.membership_function[i]) for i in xrange(len(self))]
        return '{0}({1})'.format(self.__class__.__name__, ', '.join(pairs))


class TemporalInstance(TimeInterval):
    def __init__(self, a, b):
        TimeInterval.__init__(self, a, b, 1)

    def plot(self):
        import matplotlib.pyplot as plt
        from spatiotemporal.unix_time import UnixTime

        plt.plot([UnixTime(self.a).to_datetime(), UnixTime(self.b).to_datetime()], [1, 1])
        return plt


if __name__ == '__main__':
    from utility.geometric import integral
    from scipy.stats import norm
    import matplotlib.pyplot as plt

    #event = TemporalInstance(datetime(2010, 1, 1), datetime(2011, 2, 1))
    #plt = event.plot()
    #plt.show()

    events = [
        # TemporalEvent(norm(loc=10, scale=2), norm(loc=30, scale=2), 100),
        # TemporalEvent(norm(loc=5, scale=2), norm(loc=15, scale=4), 100),

        TemporalEventPiecewiseLinear({1: 0, 2: 0.1, 3: 0.3, 4: 0.7, 5: 1}, {6: 1, 7: 0.9, 8: 0.6, 9: 0.1, 10: 0}),
        TemporalEventPiecewiseLinear({1: 0, 2: 0.1, 3: 0.3, 4: 0.7, 5: 1}, {3.5: 1, 4.5: 0.9, 8: 0.6, 9: 0.1, 10: 0})
    ]

    print type(events[0])

    print events[0] * events[1]

    for event in events:
        plt = event.plot()
        print integral(event.distribution_beginning.pdf, event.a, event.beginning)
        print event.distribution_beginning.rvs(10)
        plt.ylim(ymax=1.1)
        #plt.figure()
    plt.show()
