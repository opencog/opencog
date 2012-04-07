#from opencog.atomspace import types

import collections
import cPickle as pickle

def if_(cond, t, f):
    if cond:
        return t
    else:
        return f

import operator
def concat_lists(lists):    
    return reduce(operator.concat, lists, [])

def inplace_set_attributes(obj, **attr_values):
    obj.__dict__.update(attr_values)
    return obj

import functools
import operator

def product(seq):
    """Product of a sequence."""
    return functools.reduce(operator.mul, seq, 1)

# These can't be used in PyPy but otherwise are fine.
#def output_atoms(atomspace):
#    roots = [x for x in atomspace.get_atoms_by_type(types.Atom) if not x.incoming]
#    #return repr( map(tree_from_atom, roots) )
#    import tree
#    for tr in map(tree.tree_from_atom, roots):
#        print repr(tr)
#
#def save_trees(trees, file):
#    '''Save all of the Trees from the AtomSpace to a file. Uses FakeAtom rather than the Cython Atom class.'''
#    import tree
#    f = open(file,'w')
#    #trees = map(tree.tree_from_atom, trees.get_atoms_by_type(types.Atom))
#    #trees = map(tree.tree_from_atom, trees)
#    trees = map(tree.tree_with_fake_atoms, trees)
#    pickle.dump(trees, f)
#    f.close()
#
#def load_trees(file):
#    '''Load all of the Trees from a file. Returns them in a list.'''
#    f = open(file, 'r')
#    trees = pickle.load(f)
#    f.close()
#    return trees

def pp(x):
    """Pretty-print any collection type (or generator).
    Prints the easy-to-read version of its members."""
    if isinstance(x, dict):
        return ppdict(x)
    elif isinstance(x, set):
        return ppset(x)
    elif isinstance(x, (tuple, list) ):
        return str ( x.__class__ ( [pp(e) for e in x] ) )
    elif isinstance(x, str):
        # Python strings are iterables, so we need to prevent the
        # infinite recursion of the next call
        return str(x)
    elif isinstance(x, collections.Iterable):
        return x.__class__.__name__ + [pp(e) for e in x]
    else:
        return str(x)

# From AIMA logic.py
def ppdict(d):
    """Print the dictionary d.
    
    Prints a string representation of the dictionary
    with keys in sorted order according to their string
    representation: {a: A, d: D, ...}.
    >>> ppdict({'m': 'M', 'a': 'A', 'r': 'R', 'k': 'K'})
    '{a: A, k: K, m: M, r: R}'

    # This doctest doesn't make sense, as there are no
    # symbols with these names!
    #>>> ppdict({z: C, y: B, x: A})
    #"{x: A, y: B, z: C}"
    """
    if not len(d): return str(d)

    def format(k, v):
        return "%s: %s" % (pp(k), pp(v))

    ditems = d.items()
    ditems.sort(key=str)
    k, v = ditems[0]
    dpairs = format(k, v)
    for (k, v) in ditems[1:]:
        dpairs += (', ' + format(k, v))
    return '{%s}' % dpairs

def ppset(s):
    """Print the set s.

    >>> ppset(set(['A', 'Q', 'F', 'K', 'Y', 'B']))
    "set([A, B, F, K, Q, Y])"

    #>>> ppset(set([z, y, x]))
    #"set([x, y, z])"
    """

    slist = list(s)
    slist.sort(key=str)
    return 'set(%s)' % pp(slist)


import collections

class OrderedSet(collections.OrderedDict, collections.MutableSet):
    def __init__(self, elements=[]):
        collections.OrderedDict.__init__(self)
        collections.MutableSet.__init__(self)
        for e in elements:
            self.add(e)

    def update(self, *args, **kwargs):
        if kwargs:
            raise TypeError("update() takes no keyword arguments")

        for s in args:
            for e in s:
                 self.add(e)

    def add(self, elem):
        self[elem] = None
        
    def append(self, elem):
        self.add(elem)

    def discard(self, elem):
        self.pop(elem, None)

    def pop_last(self):
        return self.popitem(last=True)[0]

    def pop_first(self):
        return self.popitem(last=False)[0]
    
    def __le__(self, other):
        return all(e in other for e in self)

    def __lt__(self, other):
        return self <= other and self != other

    def __ge__(self, other):
        return all(e in self for e in other)

    def __gt__(self, other):
        return self >= other and self != other

    def __repr__(self):
        return 'OrderedSet([%s])' % (', '.join(map(repr, self.keys())))

    def __str__(self):
        return '{%s}' % (', '.join(map(repr, self.keys())))

    difference = property(lambda self: self.__sub__)
    difference_update = property(lambda self: self.__isub__)
    intersection = property(lambda self: self.__and__)
    intersection_update = property(lambda self: self.__iand__)
    issubset = property(lambda self: self.__le__)
    issuperset = property(lambda self: self.__ge__)
    symmetric_difference = property(lambda self: self.__xor__)
    symmetric_difference_update = property(lambda self: self.__ixor__)
    union = property(lambda self: self.__or__)

class Logger(object):
    def info(self, msg):
        #print msg
        pass
    
    def use_stdout(self, use):
        pass

log = Logger()
