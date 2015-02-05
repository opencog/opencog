"""
Example usage of the Python wrapper for the MOSES evolutionary program
learning system

Instantiates MOSES with a training dataset stored in a Python array and learns
a program representation of the XOR operation

For complete documentation on how to pass additional parameters to MOSES,
refer to the documentation at the following link:
https://github.com/opencog/opencog/blob/master/opencog/cython/opencog/pymoses.pyx

MOSES Details:
http://wiki.opencog.org/w/Meta-Optimizing_Semantic_Evolutionary_Search
"""

__author__ = 'Cosmo Harrigan'

from opencog.pymoses import *

moses = moses()

# INPUT DATA:
input_data = [[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]

output = moses.run(input=input_data, python=True)

print "\nTraining data:\n\n{0}".format(input_data)

print "\nThe following program was learned:" \
      "\n-------------------\n\n{0}".\
    format(output[0].program)

model = output[0].eval

print "-------------------\nTesting model on data:\n"

print "[0, 1]: {0}".format(model([0, 1]))
print "[1, 1]: {0}".format(model([1, 1]))

print "\n-------------------\nEquivalent Scheme program:\n"

print moses.run(input=input_data, python=False)[0].program
