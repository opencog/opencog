import numpy
from lmfit import minimize, Parameters, Parameter, report_fit
from spatiotemporal.temporal_events.trapezium import TemporalEventTrapezium

__author__ = 'keyvan'


class Fitter(object):
    def __init__(self, relations):
        self.relations = []
        for relation in 'pmoFDseSdfOMP':
            self.relations.append(relations[relation])
        self.relations = numpy.array(self.relations)

        self.params = Parameters()
        self.params.add_many(
            ('before_dist_1_beginning_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('same_dist_1_beginning_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_beginning_dist_2_beginning', None, False, None, None,
             '1 - before_dist_1_beginning_dist_2_beginning'),
            ('before_dist_1_beginning_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('same_dist_1_beginning_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_beginning_dist_2_ending', None, False, None, None,
             '1 - before_dist_1_beginning_dist_2_ending'),
            ('before_dist_1_ending_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('same_dist_1_ending_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_ending_dist_2_beginning', None, False, None, None,
             '1 - before_dist_1_ending_dist_2_beginning'),
            ('before_dist_1_ending_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('same_dist_1_ending_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_ending_dist_2_ending', None, False, None, None,
             '1 - before_dist_1_ending_dist_2_ending')
        )

    def fitness(self, params):
        model = numpy.zeros(13)

        before_dist_1_beginning_dist_2_beginning = params['before_dist_1_beginning_dist_2_beginning'].value
        same_dist_1_beginning_dist_2_beginning = params['same_dist_1_beginning_dist_2_beginning'].value
        after_dist_1_beginning_dist_2_beginning = params['after_dist_1_beginning_dist_2_beginning'].value
        before_dist_1_beginning_dist_2_ending = params['before_dist_1_beginning_dist_2_ending'].value
        same_dist_1_beginning_dist_2_ending = params['same_dist_1_beginning_dist_2_ending'].value
        after_dist_1_beginning_dist_2_ending = params['after_dist_1_beginning_dist_2_ending'].value
        before_dist_1_ending_dist_2_beginning = params['before_dist_1_ending_dist_2_beginning'].value
        same_dist_1_ending_dist_2_beginning = params['same_dist_1_ending_dist_2_beginning'].value
        after_dist_1_ending_dist_2_beginning = params['after_dist_1_ending_dist_2_beginning'].value
        before_dist_1_ending_dist_2_ending = params['before_dist_1_ending_dist_2_ending'].value
        same_dist_1_ending_dist_2_ending = params['same_dist_1_ending_dist_2_ending'].value
        after_dist_1_ending_dist_2_ending = params['after_dist_1_ending_dist_2_ending'].value

        model[0] = before_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   before_dist_1_ending_dist_2_beginning * before_dist_1_ending_dist_2_ending

        model[1] = before_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   same_dist_1_ending_dist_2_beginning * before_dist_1_ending_dist_2_ending

        model[2] = before_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * before_dist_1_ending_dist_2_ending

        model[3] = before_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * same_dist_1_ending_dist_2_ending

        model[4] = before_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * after_dist_1_ending_dist_2_ending

        model[5] = same_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * before_dist_1_ending_dist_2_ending

        model[6] = same_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * same_dist_1_ending_dist_2_ending

        model[7] = same_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * after_dist_1_ending_dist_2_ending

        model[8] = after_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * before_dist_1_ending_dist_2_ending

        model[9] = after_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                   after_dist_1_ending_dist_2_beginning * same_dist_1_ending_dist_2_ending

        model[10] = after_dist_1_beginning_dist_2_beginning * before_dist_1_beginning_dist_2_ending * \
                    after_dist_1_ending_dist_2_beginning * after_dist_1_ending_dist_2_ending

        model[11] = after_dist_1_beginning_dist_2_beginning * same_dist_1_beginning_dist_2_ending * \
                    after_dist_1_ending_dist_2_beginning * after_dist_1_ending_dist_2_ending

        model[12] = after_dist_1_beginning_dist_2_beginning * after_dist_1_beginning_dist_2_ending * \
                    after_dist_1_ending_dist_2_beginning * after_dist_1_ending_dist_2_ending

        return model - self.relations

    def minimize(self):
        solution = minimize(self.fitness, self.params)
        return [self.params[p].value for p in self.params]

if __name__ == '__main__':
    A = TemporalEventTrapezium(1, 8, 3, 7)
    B = TemporalEventTrapezium(2, 7.5, 3.8, 6)

    relations = A * B
    print relations
    f = Fitter(relations)
    print f.minimize()
