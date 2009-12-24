SET(CMAKE_BUILD_TYPE Debug)
IF (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-deprecated -Wno-unused-variable")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)


# It is now usual to use the system OpenCog header files so this is unnecessary
INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/src/util
    ${OPENCOG_SOURCE_DIR}/src/atomspace
    ${OPENCOG_SOURCE_DIR}/src/xml
    ${OPENCOG_SOURCE_DIR}/src/server
#    ${OPENCOG_SOURCE_DIR}/src/guile
#    ${OPENCOG_SOURCE_DIR}/src/adaptors/tulip
#    ${OPENCOG_SOURCE_DIR}/src/nlp/wsd
#    ${OPENCOG_SOURCE_DIR}/src/persist
#    ${OPENCOG_SOURCE_DIR}/src/query
    ${OPENCOG_SOURCE_DIR}/src/reasoning
    ${IODBC_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/atomspace
	${OPENCOG_BINARY_DIR}/opencog/server
	${OPENCOG_BINARY_DIR}/opencog/reasoning/pln
	${OPENCOG_BINARY_DIR}/opencog/util
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
ADD_CXXTEST(ConstitutedSetUTest)
ADD_CXXTEST(TableGatherUTest)
ADD_CXXTEST(BITNodeUTest)
ADD_CXXTEST(ASSOCUTest)

# At this point, PLNUTest requires libexpat, and will fail without it, since axioms 
# are loaded from XML files. See AtomSpaceWrapper.cc for details.
IF (EXPAT_FOUND)
	ADD_CXXTEST(PLNUTest)
ENDIF (EXPAT_FOUND)

ADD_SUBDIRECTORY(formulas)
ADD_SUBDIRECTORY(rules)
