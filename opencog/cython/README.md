Python bindings for OpenCog
---------------------------

## Requirements ##

* Python 2.7 - these bindings may work with earlier versions, but they have not been tested at all.
* Cython 0.14 or later. http://www.cython.org/
* Nosetests - for running unit tests.

Both Cython and Nosetests can be installed with easy_install:

 sudo easy_install cython nose

The bindings are written mostly using Cython, which allows writing
code that looks pythonic but gets compiled to C.  It also makes it
trivial to access both Python objects and C objects without using a
bunch of extra Python library API calls.

Currently the package structure looks like this:

 opencog.atomspace
 opencog.atomspace.types
 opencog.cogserver

Eventually, when other components of OpenCog are accessible, they'll
follow a similar pattern:

 opencog.rules

## Tutorial ##

This tutorial is a first look at the Python bindings. It assumes that
you've got a good grasp on the concept of the AtomSpace and the
CogServer. Oh, and it helps to know a bit of Python too!

### Setting up ###

Go through the normal process of [[building OpenCog]]. Then ensure that
the OpenCog data directory is in your Python `sys.path`. By
default, the opencog python module lives at
`/usr/local/share/opencog/python` when you do
`make install`, and you can modify your `PYTHONPATH`
environment variable to ensure Python checks that location. If you
just want to use your build dir you can use something like:

 $ export PYTHONPATH=$PYTHONPATH:/usr/local/share/opencog/python:~/src/opencog/build/opencog/cython

### MindAgents in Python ###

MindAgents modify the AtomSpace autonomously. Adding and removing atoms,
updating TruthValues or anything else. The most important part for now is
the "run" method, which gets called with the CogServer AtomSpace as a parameter
(In the past, C++ MindAgents would be passed the CogServer itself, but it
wasn't obvious to me why this is necessary).

```python
>>> import opencog.cogserver
>>> from opencog.atomspace import types
>>> class MyMindAgent(opencog.cogserver.MindAgent):
...    def run(self,atomspace):
...        atomspace.add_node(types.ConceptNode, "test")
```

This will try, every CogServer cycle, to add the ConceptNode called "test".

''Warning'': Note the opencog.cogserver.MindAgent is subclassed using the full
path. If you use:

```python
>>> from opencog.cogserver import MindAgent # wrong
```

Then you'll get MindAgent showing up in the cogserver too. I'm not currently
sure why, but supporting this is another improvement to add eventually.

To actually allow the CogServer to use this MindAgent there are several things to check:

# Place the module containing the MindAgent within one of these place:
#* <your OpenCog data directory>/python
#* a directory specified in the configuration parameter PYTHON_EXTENSION_DIRS,
#* somewhere on your PYTHONPATH.
# Ensure you are loading the libPythonModule.so CogServer plugin.
# Either use the CogServer loadpy command to load the module (you should leave off the .py extension, e.g. "loadpy my_module") or place the module name in  the configuration parameter PYTHON_PRELOAD.

Note: Yet another thing to do is allow all modules in the OpenCog Python
directories to be loaded automatically and scanned for MindAgents.

### CogServer Requests in Python ###

CogServer Requests are commands that can be issued in the shell or by other
modules in C++ code. If save the follow to a file myrequest.py:

```python
import opencog.cogserver
from opencog.atomspace import types
class MyRequest(opencog.cogserver.Request):
    def run(self,args,atomspace):
        # args is a list of strings
        atomspace.add_node(types.ConceptNode, args[0])
```

And you telnet to a running CogServer, you can run the request:

 $ telnet localhost 17001
 opencog> loadpy myrequest
 opencog> myrequest.MyRequest blah

Then the request will have added a ConceptNode with the name "blah"
