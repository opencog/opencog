
HOWTO run these tests by hand:
------------------------------

You need to set up the PYTHON path:
export PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_SOURCE_DIR}/opencog/python:${PROJECT_SOURCE_DIR}/opencog/python/opencog:${PROJECT_SOURCE_DIR}/opencog/nlp

For example:
export PYTHONPATH=build/opencog/cython:./opencog/python:./opencog/python/opencog:./opencog/nlp

Then:

nosetests -vs ${CMAKE_SOURCE_DIR}/tests/python/

If you modify the cython bindings, you may need to manually remove
some build files to get a clean rebuild.  Basically, the CMakefiles
for cython/python are buggy, and fail to rebuild when changes are made.
So:

rm build/opencog/cython/opencog/pymoses.cpp
