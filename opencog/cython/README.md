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

The OpenCog wiki contains the Python tutorial:

http://wiki.opencog.org/w/Python
