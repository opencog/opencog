
Execution and Evaluation Support
================================

The code here implements a very crude form of execution support for
the AtomSpace. Its crude, because its mostly simple, straight-forward,
and painfully slow and inefficient.

First, some reasons its slow, and ways to speed that up. Second,
some more abstract thoughts about language design.

The code is slow because function names are currently stored as strings.
Thus, each time execution needs to be done, the string needs to be
decoded.  Next, after being decoded, either the guile, python or combo
interpreters need to be entered; this takes a fair amount of time.

The string decodes could be solved by decoding the string only once,
and caching that value (memoizing it).  The cached value would be the
SCM symbol, or the PyObject that the string name refers to.  Likewise,
the arguments: currently, just handles, could also be memoized (again,
as SCM's or PyObjects).

Where should these cached values be stored? Well, obviously, the best
place seems to be the Atom itself.  The memoized function should be
stored in the GSN/GPN atom.

The problem is that this makes the Atom fatter still (every Atom would
need to have an SCM in it, a PyObject in it, an hl for Haskell...)
The alternative would be to use a map to store only the afftecte atoms.
The map should live in the atomspace, because deleted atoms would have
to be removed from the map, as well.  Thus, there would need to be a
calls

```
SCM AtomSpace::getSCM(Handle);
PyObject* AtomSpace::getPy(Handle);
```

Side effects
------------
There is also another theoretical issue: the current design assumes
that ExecutionOutputLink can have side effects, and thus, it needs
to be run ever time that it is encountered.  This is a poor choice
for good performance; we need to define a side-effect-free execution
link, and we need to define a monad when the side effects are needed.
