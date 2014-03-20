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

def list_powerset(lst):
    # the power set of the empty set has one element, the empty set
    result = [[]]
    for x in lst:
        # for every additional element in our set
        # the power set consists of the subsets that don't
        # contain this element (just take the previous power set)
        # plus the subsets that do contain the element (use list
        # comprehension to add [x] onto everything in the
        # previous power set)
        result.extend([subset + [x] for subset in result])
    return result

# These can't be used in PyPy but otherwise are fine.
def list_powerset(lst):
    # the power set of the empty set has one element, the empty set
    result = [[]]
    for x in lst:
        # for every additional element in our set
        # the power set consists of the subsets that don't
        # contain this element (just take the previous power set)
        # plus the subsets that do contain the element (use list
        # comprehension to add [x] onto everything in the
        # previous power set)
        result.extend([subset + [x] for subset in result])
    return result

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

#def format_log(offset, dsp_suffix = True, *args):
#    suffix = ""
#    if dsp_suffix:
#        #stack = inspect.stack()
#        #suffix = " -- %s %s" % (stack[1][2], stack[1][3])
#        pass
#    out =  ' ' * offset +  ' '.join(map(str, args)) + suffix
#    return out

def format_log(*args):
    out =  ' '.join(map(str, args))
    return out

# Note that this has the same name, but works differently, than the
# logger class in opencog/cython/opencog/logger.pyx
#
# Note that there is yet a third logger in
# opencog/python/dingjie/m_util.py that is like this, but colorized.
# XXX FIXME All these different versions should be merged to one
# version.
class Logger(object):
    def __init__(self, f = 'opencog-python.log'):
        self._file = open(f,'w')
        self.to_stdout = False
        self._leves = []
        self.DEBUG = 0
        self.INFO = 1
        self.WARNING = 2
        self.ERROR = 3
        #
        self.trace = False
        self.ident = 0
    
    def debug(self,msg):
        """docstring for debug"""
        if self.trace and self.DEBUG in self._leves:
            print >>self._file, "[DEBUG]:"  + msg
            if self.to_stdout:
                print "[DEBUG]:" +  msg 
    def info(self, msg):
        if self.trace and self.INFO in self._leves:
            print >>self._file, "[INFO]:"  + msg
            if self.to_stdout:
                print "[INFO]:" +  msg 
    def warning(self,msg):
        """docstring for debug"""
        if self.trace and self.WARNING in self._leves:
            print >>self._file, "[WARNING]:"  + msg
            if self.to_stdout:
                print "[WARNING]:" +  msg 
    def error(self, msg):
        if self.trace and self.ERROR in self._leves:
            print >>self._file, "[ERROR]:"  + msg
            if self.to_stdout:
                print "[ERROR]:" +  msg 
    def flush(self):
        self._file.flush()
    
    def use_stdout(self, use):
        self.to_stdout = use
    def setLevel(self, level):
        self._leves.append(level)

log = Logger()
#class Logger(object):
    #def __init__(self, f = 'opencog-python.log'):
        #self._file = open(f,'w')
    
    #def info(self, msg):
        #print >>self._file, msg
        #self._file.flush()
        ##pass
    
    #def use_stdout(self, use):
        #pass

#log = Logger()


## Note. Due to various technical annoyances, the json save/load
## functionality probably won't work atomspace_remote currently
#try:
#    from opencog.atomspace import types as t, TruthValue
#except ImportError:
#    from atomspace_remote import types as t, TruthValue
#
#import json
#from opencog.atomspace import get_type
#
#
#def save_atomspace_json(space, file='atomspace.json'):
#    f = open(file,'w')
#    for atom in space.get_atoms_by_type(t.Atom):
#        f.write(_json_from_atom(atom))
#
#def load_atomspace_json(space, file='atomspace.json'):
#    f = open(file, 'r')
#    for line in f.readlines():
#        _atom_from_json(space, line)

# repeated here because the atomspace_remote version needs to be tied to the
# Python AtomSpace internals (and this version is connected to the Cython
# AtomSpace internals)
def _atom_from_json(space, s):
    d = json.loads(s)
    
    return _atom_from_dict(space, d)
    
def _atom_from_dict(space, d):
    (type_name,) = d['type'],
    (name,) = d['name'],
    (out,) = d['outgoing'],
    (tv,) = _tv_from_dict(d['truthvalue']),
    
    out = [_atom_from_dict(space, o) for o in out]
    
    if out == []:
        out = None
    if type_name.endswith('Link'):
        name = None
    
    #print get_type(type_name), name, out, tv
    atom =space.add(get_type(type_name), name=name,
              out = out, tv = tv)
    return atom

def _json_from_atom(atom):
    d = _dict_from_atom(atom)
    return json.dumps(d) + '\r\n'
    
def _dict_from_atom(a):
    out = map(_dict_from_atom, a.out)
    d = {
        'type':a.type_name,
        'name':a.name,
        'outgoing':out,
        'truthvalue':{'simple':{'str':a.tv.mean,'count':a.tv.count}},
        # TODO set sti and lti
        }

    return d

def _tv_from_dict(d):
    stv_dict = d['simple']
    return TruthValue(stv_dict['str'], stv_dict['count'])
