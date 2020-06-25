from distutils.core import setup
from distutils.extension import Extension
from distutils.sysconfig import get_python_inc
from Cython.Distutils import build_ext
import os

# WARNING: This file is deprecated as the cython bindings are integrated with
# CMake now. This file is still useful for reference on how to use distutils
# with cython

# How can we make this configurable by CMake?
# Perhaps has a environment variable or a command line argument
opencog_library_dir = "/usr/local/lib/opencog"
# for OSX:
#opencog_library_dir = "/opt/local/lib/opencog"
# for non-installed build:
#opencog_library_dir = "../../bin/opencog/atomspace",

# Utility function to read the README file.
# Used for the long_description.  It's nice, because now 1) we have a top level
# README file and 2) it's easier to type in the README file than to put a raw
# string in below ...
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

incdir = os.path.join(get_python_inc(plat_specific=1), 'Numerical')

# This extension stuff should use info from CMake somehow...
ext = Extension(
    "opencog",                 # name of extension
    define_macros = [('MAJOR_VERSION', '0'),
                     ('MINOR_VERSION', '1')],
    sources=["opencog.pyx"],     # filename of our Cython source
    language="c++",              # this causes Cython to create C++ source

    include_dirs=[".", # needed to find local pyx/pxd files
        "../..",       # to support building in source directory
        "/usr/local/include", # For local includes
        "/opt/local/include" # For MacPorts
        ],
    libraries=["stdc++",
        # opencog libraries
        "atomspace",
        "util"
        ],
    library_dirs=[
        "/opt/local/lib", # For MacPorts
        opencog_library_dir],
    runtime_library_dirs=[opencog_library_dir]
    )

# This extension stuff should use info from CMake somehow...
helper_ext = Extension(
    "agent_finder",                 # name of extension
    define_macros = [('MAJOR_VERSION', '0'),
                     ('MINOR_VERSION', '1')],
    sources=["agent_finder.pyx"],     # filename of our Cython source
    language="c++",              # this causes Cython to create C++ source

    include_dirs=[".", # needed to find local pyx/pxd files
        "/usr/local/include", # For local includes
        "/opt/local/include" # For MacPorts
        ],
    libraries=["stdc++",
        ],
    library_dirs=[
        "/opt/local/lib", # For MacPorts
        ]
    )

setup(name = 'pyopencog',
    description = 'OpenCog Python bindings',
    author = 'Joel Pitt',
    author_email = 'joel@opencog.org',
    url = 'http://wiki.opencog.org/w/Python',
    long_description = read('README.md'),
    version = '0.1',
    classifiers=[
        "Development Status :: 3 - Alpha",
        #"Development Status :: 4 - Beta",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "License :: OSI Approved :: GNU Affero General Public License v3",
    ],
    cmdclass = {'build_ext': build_ext},
    ext_modules = [ext, helper_ext]
    )


