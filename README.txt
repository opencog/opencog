
                          OpenCog
                          -------

OpenCog is a general-purpose reasoning system.
It currently consists of only the barest of essentials.

Prerequisites:
--------------
To build and run OpenCog, the following pacakges are required:
cmake -- a build management tool -- http://www.cmake.org/
cxxtest -- a test framework -- http://cxxtest.sourceforge.net/

To build opencog:
-----------------
cd to project root dir
mkdir bin
cd bin
cmake ..
make

libs will be built into each src subdirectory.

Unit tests:
-----------
To run unit tests type (from the bin directory):

../scripts/run_utests.sh



ToDo
----
Get cmake to automatically search for and detect cxxtest
(and complain when its missing)

Some useful CMake's web sites/pages: 

 - http://www.cmake.org (main page) 
 - http://www.cmake.org/Wiki/CMake_Useful_Variables 
 - http://www.cmake.org/Wiki/CMake_Useful_Variables/Get_Variables_From_CMake_Dashboards
 - http://www.cmake.org/Wiki/CMakeMacroAddCxxTest
 - http://www.cmake.org/Wiki/CMake_HowToFindInstalledSoftware


