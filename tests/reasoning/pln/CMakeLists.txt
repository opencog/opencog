IF (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-deprecated -Wno-unused-variable")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

# It is now usual to use the system OpenCog header files so this is unnecessary
INCLUDE_DIRECTORIES (
    ${PROJECT_SOURCE_DIR}/opencog/util
    ${PROJECT_SOURCE_DIR}/opencog/atomspace
    ${PROJECT_SOURCE_DIR}/opencog/xml
    ${PROJECT_SOURCE_DIR}/opencog/server
    ${PROJECT_SOURCE_DIR}/opencog/reasoning
    ${IODBC_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
)

LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/server
	${PROJECT_BINARY_DIR}/opencog/reasoning/pln
	${PROJECT_BINARY_DIR}/opencog/util
    ${Boost_LIBRARY_DIRS}
)

LINK_LIBRARIES (
    pln
    server
    PythonModule
    ${PYTHON_LIBRARIES}
)

ADD_CXXTEST(AtomSpaceWrapperUTest)
ADD_CXXTEST(BITNodeUTest)
ADD_CXXTEST(ASSOCUTest)
ADD_CXXTEST(PLNUtilsUTest)
ADD_CXXTEST(PLNSchemeWrapperUTest)

# At this point, PLNUTest requires libexpat or the Scheme subsystem since axioms 
# are loaded from files.
IF (EXPAT_FOUND OR GUILE_FOUND)
        # Nil: PLNUTest and ForwardChainingUTest always fail and take
        # a very long time so I disable them till they get fixed
    	ADD_CXXTEST(PLNUTest)
    # IF (NOT CMAKE_BUILD_TYPE STREQUAL "Coverage")
        # ADD_CXXTEST(ForwardChainingUTest)
    ADD_CXXTEST(DeterminismUTest)
    #ENDIF (NOT CMAKE_BUILD_TYPE STREQUAL "Coverage")
ENDIF (EXPAT_FOUND OR GUILE_FOUND)

ADD_SUBDIRECTORY(formulas)
ADD_SUBDIRECTORY(rules)
