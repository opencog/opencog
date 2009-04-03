ADD_DEFINITIONS(-DNOWX)

INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AGISimSim/shared/include
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AGISimSim/server/include
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/src/AGISimSim/shared
)

ADD_EXECUTABLE(AGISimSimServer 
    src/agisim.cpp
    src/CSagent.cpp
    src/CSproxy.cpp
    src/demon.cpp
    src/sensationxml.cpp
    src/serversocket.cpp
    src/collisionsystem.cpp
    src/CSserver.cpp  src/pseudospace.cpp
    src/sensor.cpp
    src/simworld.cpp
    src/commands.cpp
    src/CSMeshTool.cpp
    src/CSworld.cpp
    src/sensation.cpp
    src/server.cpp
    src/StdAfx.cpp
    src/genericsocket.cpp
)

TARGET_LINK_LIBRARIES(AGISimSimServer 
    AGISimSimShared 
    pthread 
    ssl 
    crypto 
    boost_thread 
    dl
    ${CSOCKETS_LIBRARIES}
)

EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AGISimSim/server/simconfig.def ${OPENCOG_BINARY_DIR}/opencog/embodiment/AGISimSim/server/simconfig.def)
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy_directory ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AGISimSim/server/data ${OPENCOG_BINARY_DIR}/opencog/embodiment/AGISimSim/server/data)

