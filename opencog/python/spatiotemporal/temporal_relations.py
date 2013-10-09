from spatiotemporal.temporal_events import BaseTemporalEvent

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

    def __repr__(self):
        if len(self) == len(TEMPORAL_RELATIONS):
            return 'spatiotemporal.temporal_relations.TemporalRelation(pmoFDseSdfOMP(full))'
        if self == 'oFDseSdfO':
            return 'spatiotemporal.temporal_relations.TemporalRelation(oFDseSdfO(concur))'
        return 'spatiotemporal.temporal_relations.TemporalRelation({0}({1}))'.format(
            ''.join(self),
            ', '.join([TEMPORAL_RELATIONS[char] for char in self]))

    def __str__(self):
        return repr(self)


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
    from spatiotemporal.temporal_events import generate_random_events

    events = generate_random_events(10)
    table = create_event_relation_hashtable(events)
    print table