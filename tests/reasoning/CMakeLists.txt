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
    ${CSOCKETS_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
    ${IODBC_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/atomspace
	${OPENCOG_BINARY_DIR}/opencog/server
	${OPENCOG_BINARY_DIR}/opencog/reasoning/pln
	${OPENCOG_BINARY_DIR}/opencog/util
)

LINK_LIBRARIES (
    atomspace
    server
    util
    pln
    xml
#    ${Boost_SIGNALS_LIBRARY}
    ${GSL_LIBRARIES}
#    ${CSOCKETS_LIBRARIES}
#    ${OPENSSL_LIBRARIES}
    ${EXPAT_LIBRARIES}
    ${GUILE_LIBRARIES}
)


#ADD_CXXTEST(XMLNodeLoaderUTest)

# PLN tests require XML libraries, because libpln.so requires them because AtomSpaceWrapper
# requires them.  If AtomSpaceWrapper were refactored,this would not be an issue ... 
IF (EXPAT_FOUND)
	ADD_CXXTEST(FormulaKitUTest)
	ADD_CXXTEST(IndefinitePLNFormulaUTest)
	ADD_CXXTEST(AtomSpaceWrapperUTest)
	ADD_CXXTEST(PLNUTest)
ENDIF (EXPAT_FOUND)


