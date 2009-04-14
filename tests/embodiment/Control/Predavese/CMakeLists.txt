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

