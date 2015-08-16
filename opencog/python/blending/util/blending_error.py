__author__ = 'DongMin Kim'


# http://stackoverflow.com/questions/36932/how-can-i-represent-an-enum-in-python
def enum_simulate(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)


blending_status = enum_simulate(
    'SUCCESS',
    'IN_PROCESS',

    'UNKNOWN_ERROR',
    'PARAMETER_ERROR',

    'ERROR_IN_PREPARE_HOOK',
    'ERROR_IN_ATOMS_CHOOSER',
    'ERROR_IN_BLENDING_DECIDER',
    'ERROR_IN_BLEND_ATOM_MAKER',
    'ERROR_IN_LINK_CONNECTOR',
    'ERROR_IN_FINISH_HOOK',

    'NOT_ENOUGH_ATOMS',
    'TOO_MANY_ATOMS',
    'UNKNOWN_TYPE',
    'EMPTY_RESULT'
)


# noinspection PyUnresolvedReferences
def get_status_str(status):
    return blending_status.reverse_mapping[status]


# noinspection PyUnresolvedReferences
def is_succeed(inst):
    try:
        status = inst.last_status
        return True if status == blending_status.SUCCESS else False
    except AttributeError:
        # It doesn't have exception handling system.
        return False
