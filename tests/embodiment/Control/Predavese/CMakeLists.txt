INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/test/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/include/xerces
)

LINK_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/lib/linux/xerces
) 

SET (CMAKE_BUILD_TYPE Debug)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_EXECUTABLE (PredaveseTest  PredaveseTestExec.cc)

TARGET_LINK_LIBRARIES (PredaveseTest
    Predavese
    behavior
    comboreduct
)
IF(WIN32)
    TARGET_LINK_LIBRARIES (PredaveseTest winmm)
ENDIF(WIN32)

