

Debugging notes
===============

Cython modules are installed here:
```
/usr/local/share/opencog/python/opencog
```

Get to the cogserver prompt like this:
```
rlwrap telnet localhost 17001
```
and then enter the python interpreter by saying `py`, or the scheme
interpreter by saying `scm`.

From the python prompt, the following should list the python
opencog modules:
```
help('opencog')
```

The following is standard python, but will crash unless run inside
the cogserver: Viz
```
import opencog.cogserver
print dir(opencog.cogserver)

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
else it returns nil.

```
rlwrap telnet localhost 17001
(cog-node 'ConceptNode "green light")
```
