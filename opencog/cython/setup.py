from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

# This extension stuff should use info from CMake somehow...
ext = Extension(
    "opencog",                 # name of extension
    define_macros = [('MAJOR_VERSION', '0'),
                     ('MINOR_VERSION', '5')],
    sources=["atomspace.pyx"],     # filename of our Cython source
    language="c++",              # this causes Cython to create C++ source

    include_dirs=[".", # needed to find local pyx/pxd files
        "../..",       # to eventually be replaced with the prefix for
                       # OpenCog's headers
        "/opt/local/include" # For the rest...
        ],
    libraries=["stdc++",
        "boost_system-mt","boost_thread-mt", # boost dependencies
        "atomspace"    # OpenCog libraries
        ],
    extra_link_args=["-L../../bin/opencog/atomspace","-L/opt/local/lib"],
    )

setup(name = 'pyopencog',
    description = 'OpenCog Python bindings',
    author = 'Joel Pitt',
    author_email = 'joel@opencog.org',
    url = 'http://wiki.opencog.org/w/Python',
    long_description = '''
Python bindings for OpenCog, the General Artificial Inteligence (GAI)
research framework. http://opencog.org/
''',
    version = '0.5',
    cmdclass = {'build_ext': build_ext},
    ext_modules = [ext]
    )


