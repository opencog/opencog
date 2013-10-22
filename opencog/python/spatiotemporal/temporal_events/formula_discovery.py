from spatiotemporal.temporal_events import TemporalEventSimple

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
