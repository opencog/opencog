from temporal.temporal_events import FormulaCreator
from temporal.temporal_events.relation_formulas import TemporalRelation
import numpy as np

__author__ = 'keyvan'

dist_1_beginning = 'dist_1_beginning'
dist_1_ending = 'dist_1_ending'
dist_2_beginning = 'dist_2_beginning'
dist_2_ending = 'dist_2_ending'
all_relations = TemporalRelation.all_relations


class Individual(dict):
    combinations = [
        (dist_1_beginning, dist_2_beginning),
        (dist_1_beginning, dist_2_ending),
        (dist_1_ending, dist_2_beginning),
        (dist_1_ending, dist_2_ending)
    ]

    def __init__(self, relations,
                 before_dist_1_beginning_dist_2_beginning, same_dist_1_beginning_dist_2_beginning,
                 before_dist_1_beginning_dist_2_ending, same_dist_1_beginning_dist_2_ending,
                 before_dist_1_ending_dist_2_beginning, same_dist_1_ending_dist_2_beginning,
                 before_dist_1_ending_dist_2_ending, same_dist_1_ending_dist_2_ending):
        self.relations = relations

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

    def fitness(self):
        actual = self.relations
        approximation = self.formula.calculate_relations()
        goal, solution = [], []
        for relation in all_relations:
            goal.append(actual[relation])
            solution.append(approximation[relation])

        goal, solution = np.array(goal), np.array(solution)
        return np.linalg.norm(goal - solution)
