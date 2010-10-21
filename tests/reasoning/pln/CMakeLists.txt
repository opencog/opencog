#SET(CMAKE_BUILD_TYPE Debug)
IF (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-deprecated -Wno-unused-variable")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

# It is now usual to use the system OpenCog header files so this is unnecessary
INCLUDE_DIRECTORIES (
    ${PROJECT_SOURCE_DIR}/src/util
    ${PROJECT_SOURCE_DIR}/src/atomspace
    ${PROJECT_SOURCE_DIR}/src/xml
    ${PROJECT_SOURCE_DIR}/src/server
#    ${PROJECT_SOURCE_DIR}/src/guile
#    ${PROJECT_SOURCE_DIR}/src/adaptors/tulip
#    ${PROJECT_SOURCE_DIR}/src/nlp/wsd
#    ${PROJECT_SOURCE_DIR}/src/persist
#    ${PROJECT_SOURCE_DIR}/src/query
    ${PROJECT_SOURCE_DIR}/src/reasoning
    ${IODBC_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
)

LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/server
	${PROJECT_BINARY_DIR}/opencog/reasoning/pln
	${PROJECT_BINARY_DIR}/opencog/util
#	/opt/boost_1_38_0/lib
	${Boost_SIGNALS_LIBRARY}
)

LINK_LIBRARIES (
    atomspace
    server
    util
    pln
#    ${Boost_SIGNALS_LIBRARY}
    ${GSL_LIBRARIES}
    ${EXPAT_LIBRARIES}
    ${GUILE_LIBRARIES}
)


# tests that require XML libraries.
# IF (EXPAT_FOUND)
#ADD_CXXTEST(XMLNodeLoaderUTest)
# ENDIF (EXPAT_FOUND)

ADD_CXXTEST(AtomSpaceWrapperUTest)
ADD_CXXTEST(BITNodeUTest)
ADD_CXXTEST(ASSOCUTest)
ADD_CXXTEST(PLNUtilsUTest)

# At this point, PLNUTest requires libexpat or the Scheme subsystem since axioms 
# are loaded from files.
IF (EXPAT_FOUND OR GUILE_FOUND)
	ADD_CXXTEST(PLNUTest)
    IF (NOT CMAKE_BUILD_TYPE STREQUAL "Coverage")
        # ForwardChaining takes forever with coverage reporting
        # It should really be split into smaller quicker tests
        ADD_CXXTEST(ForwardChainingUTest)
    ENDIF (NOT CMAKE_BUILD_TYPE STREQUAL "Coverage")
ENDIF (EXPAT_FOUND OR GUILE_FOUND)

ADD_SUBDIRECTORY(formulas)
ADD_SUBDIRECTORY(rules)
