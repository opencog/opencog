__author__ = 'sebastian'

from temporal.temporal_events.trapezium import TemporalEventTrapezium

all_relations = "pmoFDseSdfOMP"

a = TemporalEventTrapezium(1, 12, 4, 8)
b = TemporalEventTrapezium(9, 17, 13, 15)

temporal_relations = a * b
print(temporal_relations)
print(temporal_relations.to_list())
for relation in all_relations:
    print(relation, temporal_relations[relation])
