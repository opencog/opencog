
HOWTO run these tests by hand:
------------------------------

You need to set up the PYTHON path:
export PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython

For example:
export PYTHONPATH=build/opencog/cython

Then:

nosetests -vs ${CMAKE_SOURCE_DIR}/tests/cython/
nosetests -vs ${CMAKE_SOURCE_DIR}/tests/cython/atomspace/
nosetests -vs ${CMAKE_SOURCE_DIR}/tests/cython/guile/
nosetests -vs ${CMAKE_SOURCE_DIR}/tests/cython/server/
nosetests -vs ${CMAKE_SOURCE_DIR}/tests/cython/moses/


If you modify the cython bindings, you may need to manually remove
some build files to get a clean rebuild.  Basically, the CMakefiles
for cython/python are buggy, and fail to rebuild when changes are made.
So:

rm build/opencog/cython/opencog/pymoses.cpp
