Python Examples
===============
There are more basic examples in the atomspace repository.

To run these examples, you must first set the `PYTHONPATH` to the
opencog cython build directory.  In most cases, the following will
suffice:

```
export PYTHONPATH=$PYTHONPATH:/usr/local/share/opencog/python:../../build/opencog/cython
```

The cogserver cython modules are installed here:
```
/usr/local/share/opencog/python/opencog
```

From the python prompt, the following should list the python
opencog modules:
```
help('opencog')
```
The contents of a single module can be viewed by saying, for example:
```
import opencog.cogserver
print dir(opencog.cogserver)
```

## cogserver
Many major software packages include a built-in python interpreter, and
the cogserver is no exception. The reason for placing an interpreter
inside an application, instead of pretending that the application is a
library, is because this allows a far greater range of functions to be
implemented.

Most importantly, the cogserver allows multiple simultaneous threads
to be running and performing compute tasks or waitng on I/O, all working
in parallel. This is fundamentally impossible with python alone, in part
due to Python's GIL, but also for other technical reasons.  Thus, to get
all the various cogserver features, you have to use it as a server, not
a library.

The server has a single, global built-in AtomSpace. It can be obtained
with the python call `get_server_atomspace()`.  This is the atomspace
that should be used for all processing; failure to use it will lead to
you to ask: "where have all my atoms gone?".

Try the following example. First, start the cogserer:
```
cogserver
```

Get to the cogserver network shell prompt like this:
```
rlwrap telnet localhost 17001
```
and then enter the python interpreter by saying `py`. You can also get
to the scheme interpreter by saying `scm`.   From the python prompt,
enter the below. Its valid python, but several of the steps will not
work outside of the cogserver; in particular `get_server_atomspace`
works only inside the cogserver.

```
from opencog.atomspace import AtomSpace
from opencog.type_constructors import *
from opencog.cogserver import get_server_atomspace

atomspace = get_server_atomspace()
set_type_ctor_atomspace(atomspace)
ConceptNode("green light")
```
Then, verify that the ConceptNode created above is actually in the
cogserver atomspace. The scheme function `cog-node` returns a value
if and only if the indicated atom already exists in the atomspace;
else it returns nil. This can be tested from the same terminal console,
or from another:

```
rlwrap telnet localhost 17001
(cog-node 'ConceptNode "green light")
```
This should display `(ConceptNode "green light")` indicating that the
atom created by python is visible to scheme.
