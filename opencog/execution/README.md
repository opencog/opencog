
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

Non-string values
-----------------
Some node types need to store one or more non-string values.  The
NumberNode here is a rough prototype for how that could be done. By
storing a double-precision floating point number (i.e. "caching" it)
it can potentially avoid string->double and double->string conversions
every time that it is accessed.  There are several caveats:

 * This works only if all possible Handles and AtomPtr's actually
   point to an instance of the NumberNode class, instead of an
   instance of a Node class, with the type set to NUMBER_NODE.

 * The above can happen only if the AtomSpace itself is careful to
   always work with instances of NumberNode, as it is the final
   arbiter of what a Handle points to.

 * The creation of a NumberNode is still "heavyweight": one must
   crate and initialize the Atom class (which takes time); one
   must also initialize the Node class, and to do that, one must
   convert the float pt. number to a string. The string is needed
   to allow the atom to be held in AtomSpace indexes.

 * The NumberNode must be inserted into the AtomSpace. This is a
   particularly time-consuming process, require multiple lookups and
   checks and validations, insertion into multiple indexes of various
   sorts. This far exceeds the cost of initializing the atom itself.

After the above is all done, the actual float pt. value in the
NumberNode can be accessed quickly enough ... but how often is that
really needed?  Was it really worth the additional complexity? Given
the other high costs shown above, does it result in any actual speedup?

For the above reasons, it is not at all clear that having a NumberNode
class is actually a good idea.  There could be a better way...


Side effects
------------
There is also another theoretical issue: the current design assumes
that ExecutionOutputLink can have side effects, and thus, it needs
to be run ever time that it is encountered.  This is a poor choice
for good performance; we need to define a side-effect-free execution
link, and we need to define a monad when the side effects are needed.

Demo
----
Example:
```
  (define two (NumberNode 2))
  (define plu (PlusLink two two))
  (cog-execute! plu)
```
Some other expressions:
```
  (cog-execute! (PlusLink (NumberNode 3) (NumberNode 2)))
  (cog-execute! (TimesLink (NumberNode 3) (NumberNode 2)))
  (cog-execute! (TimesLink (NumberNode 4)
     (PlusLink (NumberNode 5) (NumberNode 1))))
```
Inequalities:
```
  (define g (GreaterThanLink (NumberNode 3) (NumberNode 2)))
  (cog-evaluate! g)

  (cog-evaluate! (GreaterThanLink (NumberNode 3) (NumberNode 42)))

  (cog-evaluate! (GreaterThanLink (NumberNode 3)
     (PlusLink (NumberNode 6) (NumberNode 7))))

  (cog-evaluate! (NotLink (GreaterThanLink (NumberNode 3)
     (PlusLink (NumberNode 6) (NumberNode 7)))))
```
