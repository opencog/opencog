# cython/distutils can only handle a single file as the source for a python module
# since it is helpful to be able to split the binding code into separate files,
# we just include them here

# note that the ordering of include statements may influence whether things work or not

include "classserver.pyx"
include "atomspace_details.pyx"
include "atom.pyx"
include "spacetime.pyx"

