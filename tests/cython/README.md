
HOWTO run these tests by hand:
------------------------------

You need to set up the PYTHON path:
export PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython

For example:
export
PYTHONPATH=build/opencog/cython:opencog/python:tests/cython/openpsi:tests/nlp/anaphora:opencog/nlp/anaphora

You also need to specify the library path:
export LD_LIBRARY_PATH=build/opencog/cython

Then, from the project root directory:

nosetests -vs tests/cython/openpsi/
nosetests -vs tests/nlp/anaphora

If you modify the cython bindings, you may need to manually remove
some build files to get a clean rebuild.  Basically, the CMakefiles
for cython/python are buggy, and fail to rebuild when changes are made.
So, for example:

rm build/opencog/cython/opencog/cogserver.cpp
