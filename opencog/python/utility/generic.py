__author__ = 'Keyvan'

from opencog.atomspace import TruthValue

DEFAULT_TRUTH_VALUE = TruthValue(1, 1)


def convert_dict_to_sorted_lists(dictionary):
    keys = sorted(dictionary)
    return keys, [dictionary[key] for key in keys]
