
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

Where should these cached values be stored? Well, obviously, in the
Atom itself.  Either that, or the Handle, but the Atom makes more sense.
The memoized function should be stored in the GSN/GPN atom.

At this time, (March 2015) we don't have a good way of caching the
PyObject or the SCM in an atom.  Note that a partial work-around is to
store the SCM and PyObject caches in the C++ class ExecutionOutputLink,
and implement some resolution mechanism so that the AtomSpace stores
a pointer to the C++ ExecutionOutputLink class, instead of just a plain
C++ Link class.  However, in the long run, storing the memoized SCM or
PyObject with the Atom, in general, seems like a better idea.


Side effects
------------
There is also another theoretical issue: the current design assumes
that ExecutionOutputLink can have side effects, and thus, it needs
to be run ever time that it is encountered.  This is a poor choice
for good performance; we need to define a side-effect-free execution
link, and we need to define a monad when the side effects are needed.
