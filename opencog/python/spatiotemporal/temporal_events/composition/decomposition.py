from spatiotemporal.temporal_events import RelationFormulaConvolution, FormulaCreator

__author__ = 'keyvan'

dist_1_beginning = 'dist_1_beginning'
dist_1_ending = 'dist_1_ending'
dist_2_beginning = 'dist_2_beginning'
dist_2_ending = 'dist_2_ending'


class Individual(dict):
    combinations = [
        (dist_2_beginning, dist_2_beginning),
        (dist_1_beginning, dist_2_ending),
        (dist_2_beginning, dist_2_ending),
        (dist_1_ending, dist_2_ending)
    ]

    def __init__(self, before_dist_1_beginning_dist_2_beginning, same_dist_1_beginning_dist_2_beginning,
                 before_dist_1_beginning_dist_2_ending, same_dist_1_beginning_dist_2_ending,
                 before_dist_1_ending_dist_2_beginning, same_dist_1_ending_dist_2_beginning,
                 before_dist_1_ending_dist_2_ending, same_dist_1_ending_dist_2_ending):
        self[dist_1_beginning, dist_2_beginning] = (
            before_dist_1_beginning_dist_2_beginning,
            same_dist_1_beginning_dist_2_beginning
        )
        self[dist_1_beginning, dist_2_ending] = (
            before_dist_1_beginning_dist_2_ending,
            same_dist_1_beginning_dist_2_ending
        )
        self[dist_1_ending, dist_2_beginning] = (
            before_dist_1_ending_dist_2_beginning,
            same_dist_1_ending_dist_2_beginning
        )
        self[dist_1_ending, dist_2_ending] = (
            before_dist_1_ending_dist_2_ending,
            same_dist_1_ending_dist_2_ending
        )

        self.formula = FormulaCreator(self)
    
    def compare(self, dist_1_key=dist_1_beginning, dist_2_key=dist_2_beginning):
        before, same = self[dist_1_key, dist_2_key]
        return before, same, 1 - before

class Population(list):
    def __init__(self):
        self.generation = 1