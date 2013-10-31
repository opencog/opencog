__author__ = 'keyvan'


def dim(structure):
    return structure.__dim__()


def subsets_of(collection, subsets_type=set):
    subsets = lambda x: [subsets_type([y for j, y in enumerate(set(x)) if (i >> j) & 1])
                         for i in range(2 ** len(set(x)))]
    for subset in subsets(collection):
        yield subset