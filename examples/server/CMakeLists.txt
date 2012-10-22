
# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_EXECUTABLE (derivedserver
    DerivedCogServer.cc
    DerivedMain.cc
)

TARGET_LINK_LIBRARIES (derivedserver
    server
    ${Boost_SIGNALS_LIBRARY}
)
