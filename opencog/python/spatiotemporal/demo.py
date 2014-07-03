__author__ = 'sebastian'

from spatiotemporal.temporal_events.trapezium import TemporalEventTrapezium
from spatiotemporal.temporal_events.relation_formulas import FormulaCreator
from spatiotemporal.temporal_events.composition.non_linear_least_squares import DecompositionFitter

import matplotlib.pyplot as plt


all_relations = "pmoFDseSdfOMP"

a = TemporalEventTrapezium(1, 12, 4, 8)
b = TemporalEventTrapezium(9, 17, 13, 15)

# compute relations between events
temporal_relations = a * b
print("Relations: {0}".format(temporal_relations.to_list()))
# print degree for every relation
for relation in all_relations:
    print(relation, temporal_relations[relation])

# plot events
a.plot(show_distributions=True).ylim(ymin=-0.1, ymax=1.1)
b.plot(show_distributions=True).figure()
plt.show()

# from the 13 relations, learns parameters for all combinations of the
# before, same, and after relationships between the beginning and
# ending distributions of the two intervals
formula = FormulaCreator(DecompositionFitter(temporal_relations))
# from these relationships, computes the 13 relations again
relations_estimate = formula.calculate_relations()
print("Estimated relations: {0}".format(relations_estimate.to_list()))