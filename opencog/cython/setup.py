from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

# This extension stuff should use info from CMake somehow...
ext = Extension(
    "opencog",                 # name of extension
    ["atomspace.pyx"],     # filename of our Cython source
    language="c++",              # this causes Cython to create C++ source
    include_dirs=["../..","/opt/local/include"], # usual stuff
    libraries=["stdc++",
        "boost_system-mt","boost_thread-mt",
        "atomspace"],             # ditto
    extra_link_args=["-L../../bin/opencog/atomspace","-L/opt/local/lib"],       # if needed
    )

setup(
    cmdclass = {'build_ext': build_ext},
    ext_modules = [ext]
    )

