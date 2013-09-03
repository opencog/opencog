# To build, use the following command: python setup.py build_ext --inplace -I ~/opencog/src/

from distutils.core import setup
from Cython.Build import cythonize

setup(
    ext_modules = cythonize(
    "moses.pyx",                 # our Cython source
    language="c++",             # generate C++ code
  ))