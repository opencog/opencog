from opencog.atomspace import types

import collections

def if_(cond, t, f):
    if cond:
        return t
    else:
        return f

def output_atoms(atomspace):
    roots = [x for x in atomspace.get_atoms_by_type(types.Atom) if not x.incoming]
    #return repr( map(tree_from_atom, roots) )
    import tree
    for tr in map(tree.tree_from_atom, roots):
        print repr(tr)

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
