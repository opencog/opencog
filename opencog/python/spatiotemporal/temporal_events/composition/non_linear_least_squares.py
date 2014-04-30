from math import fabs
import numpy
from lmfit import minimize, Parameters

__author__ = 'keyvan'

_keys = ['beginning', 'ending']


class DecompositionFitter(object):
    combinations = [(dist_1_key, dist_2_key) for dist_1_key in _keys for dist_2_key in _keys]

    def __init__(self, relations):
        self.data = relations.to_vector()

        self.params = Parameters()
        self.params.add_many(
            ('before_dist_1_beginning_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('similarity_dist_1_beginning_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_beginning_dist_2_beginning', None, False, None, None,
             '1 - before_dist_1_beginning_dist_2_beginning'),
            ('before_dist_1_beginning_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('similarity_dist_1_beginning_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_beginning_dist_2_ending', None, False, None, None,
             '1 - before_dist_1_beginning_dist_2_ending'),
            ('before_dist_1_ending_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('similarity_dist_1_ending_dist_2_beginning', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_ending_dist_2_beginning', None, False, None, None,
             '1 - before_dist_1_ending_dist_2_beginning'),
            ('before_dist_1_ending_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('similarity_dist_1_ending_dist_2_ending', 0.5, True, 0.0, 1.0, None),
            ('after_dist_1_ending_dist_2_ending', None, False, None, None,
             '1 - before_dist_1_ending_dist_2_ending')
        )

        minimize(self.fitness, self.params)

        for param_key in self.params:
            self.params[param_key].value = round(self.params[param_key].value, 6)

    def fitness(self, params):
        model = numpy.zeros(13)

        before_dist_1_beginning_dist_2_beginning = params['before_dist_1_beginning_dist_2_beginning'].value
        similarity_dist_1_beginning_dist_2_beginning = params['similarity_dist_1_beginning_dist_2_beginning'].value
        after_dist_1_beginning_dist_2_beginning = params['after_dist_1_beginning_dist_2_beginning'].value
        before_dist_1_beginning_dist_2_ending = params['before_dist_1_beginning_dist_2_ending'].value
        similarity_dist_1_beginning_dist_2_ending = params['similarity_dist_1_beginning_dist_2_ending'].value
        after_dist_1_beginning_dist_2_ending = params['after_dist_1_beginning_dist_2_ending'].value
        before_dist_1_ending_dist_2_beginning = params['before_dist_1_ending_dist_2_beginning'].value
        similarity_dist_1_ending_dist_2_beginning = params['similarity_dist_1_ending_dist_2_beginning'].value
        after_dist_1_ending_dist_2_beginning = params['after_dist_1_ending_dist_2_beginning'].value
        before_dist_1_ending_dist_2_ending = params['before_dist_1_ending_dist_2_ending'].value
        similarity_dist_1_ending_dist_2_ending = params['similarity_dist_1_ending_dist_2_ending'].value
        after_dist_1_ending_dist_2_ending = params['after_dist_1_ending_dist_2_ending'].value

        same_dist_1_beginning_dist_2_beginning = similarity_dist_1_beginning_dist_2_beginning * (
            1 - fabs(before_dist_1_beginning_dist_2_beginning - after_dist_1_beginning_dist_2_beginning)
        )

        same_dist_1_beginning_dist_2_ending = similarity_dist_1_beginning_dist_2_ending * (
            1 - fabs(before_dist_1_beginning_dist_2_ending - after_dist_1_beginning_dist_2_ending)
        )

        same_dist_1_ending_dist_2_beginning = similarity_dist_1_ending_dist_2_beginning * (
            1 - fabs(before_dist_1_ending_dist_2_beginning - after_dist_1_ending_dist_2_beginning)
        )

        same_dist_1_ending_dist_2_ending = similarity_dist_1_ending_dist_2_ending * (
            1 - fabs(before_dist_1_ending_dist_2_ending - after_dist_1_ending_dist_2_ending)
        )

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

        return model - self.data

    def compare(self, dist_1_key='beginning', dist_2_key='beginning'):
        before = self.params['before_dist_1_' + dist_1_key + '_dist_2_' + dist_2_key].value
        after = self.params['after_dist_1_' + dist_1_key + '_dist_2_' + dist_2_key].value
        similarity = self.params['similarity_dist_1_' + dist_1_key + '_dist_2_' + dist_2_key].value
        # before, similarity, after = round(before, 6), round(similarity, 6), round(after, 6)
        same = similarity * (1 - fabs(before - after))
        return before, same, after

    def get_composition_data(self):
        data = []
        for key in self.combinations:
            before, same, after = self.compare(*key)
            data.append(before)
            data.append(same)
        return data

    def check(self):
        from spatiotemporal.temporal_events import FormulaCreator
        print self.data
        print FormulaCreator(self).calculate_relations().to_vector()
        print

if __name__ == '__main__':
    from spatiotemporal.temporal_events import FormulaCreator
    from spatiotemporal.temporal_events.trapezium import generate_random_events
    for i in xrange(50):
        A, B = generate_random_events(2)
        relations = A * B
        formula = FormulaCreator(DecompositionFitter(relations))
        print relations.to_list()
        relations_estimate = formula.calculate_relations()
        print relations_estimate.to_list()
        print relations.to_vector() - relations_estimate.to_vector()
        print