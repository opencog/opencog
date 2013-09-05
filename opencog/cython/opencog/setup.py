# setup.py file, from http://stackoverflow.com/questions/16993927/using-cython-to-link-python-to-a-shared-library

import os
import shutil

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

# clean previous build
for root, dirs, files in os.walk(".", topdown=False):
    for name in files:
        if (name.startswith("myext") and not(name.endswith(".pyx") or name.endswith(".pxd"))):
            os.remove(os.path.join(root, name))
    for name in dirs:
        if (name == "build"):
            shutil.rmtree(name)

# build "pymoses.so" python extension to be added to "PYTHONPATH" afterwards...
setup(
    cmdclass = {'build_ext': build_ext},
    ext_modules = [
    Extension("pymoses",
              sources=["pymoses.pyx"],
              language="c++",                   # remove this if C and not C++
              extra_compile_args=["-fopenmp", "-O3", "-std=c++0x",
                                  "-I../../learning/moses/service/",
                                  "-I../../../",
                                  "-I../../../DEPENDENCIES/python2.7/inc",
                                  "-I../../../DEPENDENCIES/gsl-1.15]",
                                  ],
              extra_link_args=["-lmoses_service", "-L/home/cosmo/opencog/src/qtbin/opencog/learning/moses/service/",
                               "-lmoses_exec", "-L/home/cosmo/opencog/src/qtbin/opencog/learning/moses/main/",
                               "-lmoses", "-L/home/cosmo/opencog/src/qtbin/opencog/learning/moses/",
                               "-lcogutil", "-L/home/cosmo/opencog/src/qtbin/opencog/util/",
                               "-lcomboreduct", "-L/home/cosmo/opencog/src/qtbin/opencog/comboreduct/",
                               "-lfeature_selection", "-L/home/cosmo/opencog/src/qtbin/opencog/learning/feature-selection/",
                               "-lboost_program_options", "-lboost_thread",
                               "-I/usr/lib/openmpi/include", "-I/usr/lib/openmpi/include/openmpi", "-pthread", "-L/usr/lib/openmpi/lib", "-lmpi_cxx", "-lmpi", "-lopen-rte",
                               "-lopen-pal", "-ldl", "-Wl,-export-dynamic", "-lnsl", "-lutil", "-lm", "-ldl",
                               ],
              )
    ]
)