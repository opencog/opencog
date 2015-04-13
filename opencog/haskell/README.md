
Haskell FFI for OpenCog
-----------------------

Someday in the distant future, this directory will contain Haskell
bindings for the OpenCog C++ AtomSpace, and other parts of OpenCog.
Currently, it is a sandbox for toys and badly-implemented ideas.


Notes:
------
There is going to be friction betweeen Cabal and CMake.

CMake infrastructure:
https://bitbucket.org/arrowdodger/cmake-findcabal/src
https://github.com/kvanberendonck/cmake-haskell

The github version claims to be a better rewrite of the bitbucket
version.

Maybe should not use Cabal at first ... but, on the other hand, I cannot
get CMake to figure out what the .hs extension means ...

Random quotes from the net:
"Cabal is currently the only tool that can realistically be used to
properly build and install Haskell packages due to the great complexity
involved with getting all the details right"
