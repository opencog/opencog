ADD_LIBRARY(AGISimSimShared
    src/simconfig.cpp
    src/utils.cpp
    src/log.cpp
    src/remoteobject.cpp
    src/property.cpp
    src/XMLNode.cpp
)

INCLUDE_DIRECTORIES(
   ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AGISimSim/shared/include
   ${OPENCOG_SOURCE_DIR}/include
)

