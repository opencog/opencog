from spatiotemporal.temporal_events import TemporalEventSimple
from spatiotemporal.temporal_events.trapezium import generate_random_events

__author__ = 'keyvan'


def assert_is_instance(instance_or_iterable):
    try:
        for instance in instance_or_iterable:
            assert isinstance(instance, TemporalEventSimple)
    except:
        assert isinstance(instance_or_iterable, TemporalEventSimple)


def is_in_start_interval_of(time_step, instance):
    if instance.a >= time_step:
        if instance.a + instance.precision <= time_step:
            return True
    return False


def is_in_middle_interval_of(time_step, instance):
    if instance.a + instance.precision > time_step:
        if instance.b - instance.precision < time_step:
            return True
    return False


def is_in_finish_interval_of(time_step, instance):
    if instance.b - instance.precision >= time_step:
        if instance.b <= time_step:
            return True
    return False


def precedes(instance_1, instance_2):
    if instance_1.b > instance_2.a:
        return True
    return False


def preceded_by(instance_1, instance_2):
    return precedes(instance_2, instance_1)


def overlaps(instance_1, instance_2):
    return instance_2.b > instance_1.a and is_in_middle_interval_of(instance_2.a, instance_1)


def overlapped_by(instance_1, instance_2):
    return overlaps(instance_2, instance_1)


def during(instance_1, instance_2):
    return is_in_middle_interval_of(instance_1.a, instance_2) and is_in_middle_interval_of(instance_1.b, instance_2)


def contains(instance_1, instance_2):
    return during(instance_2, instance_1)


def equals(instance_1, instance_2):
    return is_in_start_interval_of(instance_1.a, instance_2) and is_in_finish_interval_of(instance_1.b, instance_2) or\
           is_in_start_interval_of(instance_2.a, instance_1) and is_in_finish_interval_of(instance_2.b, instance_1)


def starts(instance_1, instance_2):
    return (is_in_start_interval_of(instance_1.a, instance_2) or
            is_in_start_interval_of(instance_2.a, instance_1)) and\
           (is_in_middle_interval_of(instance_1.b, instance_2) or
            is_in_start_interval_of(instance_1.a, instance_2))


def started_by(instance_1, instance_2):
    return starts(instance_2, instance_1)


def finishes(instance_1, instance_2):
    return (is_in_finish_interval_of(instance_1.b, instance_2) or
            is_in_finish_interval_of(instance_2.b, instance_1)) and\
           (is_in_middle_interval_of(instance_1.a, instance_2) or
            is_in_finish_interval_of(instance_1.a, instance_2))


def finished_by(instance_1, instance_2):
    return finishes(instance_2, instance_1)


def meets(instance_1, instance_2):
    return not finished_by(instance_1, instance_2) and is_in_finish_interval_of(instance_2.a, instance_1)


def met_by(instance_1, instance_2):
    return meets(instance_2, instance_1)


INSTANCE_FORMULAS = {
    'p': precedes,
    'm': meets,
    'o': overlaps,
    'F': finished_by,
    'D': contains,
    's': starts,
    'e': equals,
    'S': started_by,
    'd': during,
    'f': finishes,
    'O': overlapped_by,
    'M': met_by,
    'P': preceded_by
}


if __name__ == '__main__':
    size = 10
    events = generate_random_events(2)
    instances = [[], []]

    for i, event in enumerate(events):
        plt = event.plot()
        for j in xrange(size):
            instances[i].append(event.instance())

    degrees = {}
    for predicate in INSTANCE_FORMULAS:
        degrees[predicate] = float(0)

    plt.ylim(ymax=1.1)
    instances[0][0].plot()
    instances[1][0].plot()

    for instance_1, instance_2 in ((i, j) for i in instances[0] for j in instances[1]):
        #plt.ylim(ymax=1.1)
        #instance_1.plot()
        #instance_2.plot()
        #plt.show()
        fds = 2 + 3
        for predicate in INSTANCE_FORMULAS:
            if INSTANCE_FORMULAS[predicate](instance_1, instance_2) is True:
                degrees[predicate] += 1
        #plt.clf()
        #events[0].plot()
        #events[1].plot()

    
    for predicate in degrees:
        degrees[predicate] /= size ** 2

    print degrees

    plt.show()