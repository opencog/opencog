from spatiotemporal.temporal_events import TemporalEventLinearPiecewise

__author__ = 'keyvan'

from temporal_formulas import *

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


def temporal_relation_between(temporal_event_1, temporal_event_2):
    temporal_event_1 = TemporalEventLinearPiecewise(1, 10, 3, 8)
    temporal_event_2 = TemporalEventLinearPiecewise(8, 16, 10, 14)

    ls_te1_times = temporal_event_1.to_list()
    ls_te2_times = temporal_event_2.to_list()
    ls_te1_certainties = temporal_event_1.membership_function()
    ls_te2_certainties = temporal_event_2.membership_function()

    ls_sum_times = ls_te1_times + ls_te2_times
    ls_sum_certainties = []
    for time_step in ls_sum_times:
        ls_sum_certainties.append(
            temporal_event_1.membership_function(time_step) + temporal_event_2.membership_function(time_step))

    import matplotlib.pyplot as plt

    plt.plot(ls_te1_times, ls_te1_certainties)
    plt.plot(ls_te2_times, ls_te2_certainties)
    plt.plot(ls_sum_times, ls_sum_certainties)
    plt.show()

    result = {}

    a = temporal_event_1.to_dict()
    b = temporal_event_2.to_dict()
    result['p'] = beforeFormula(a, b)
    print result['p']


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
    #from spatiotemporal.temporal_events import generate_random_events, BaseTemporalEvent, TemporalEventLinearPiecewise

    #events = generate_random_events(10)
    #table = create_event_relation_hashtable(events)
    #print table

    temporal_relation_between(1, 2)