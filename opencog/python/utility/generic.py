from xapian import InvalidOperationError

__author__ = 'Keyvan'

from opencog.atomspace import TruthValue

DEFAULT_TRUTH_VALUE = TruthValue(1, 1)


def subsets_of_len_two(seq):
    indexed_seq = list(seq)
    length = len(indexed_seq)
    for i in xrange(length):
        for j in xrange(i + 1, length):
            yield (indexed_seq[i], indexed_seq[j])


def convert_dict_to_sorted_lists(dictionary):
    keys = sorted(dictionary)
    return keys, [dictionary[key] for key in keys]