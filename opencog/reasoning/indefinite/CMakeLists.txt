SET(CMAKE_BUILD_TYPE Debug)

ADD_LIBRARY (IndefinitePLNFormulas STATIC
	IndefinitePLNFormulas.cc
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/xml
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/guile
	${OPENCOG_SOURCE_DIR}/src/nlp/wsd
	${OPENCOG_SOURCE_DIR}/src/persist
	${OPENCOG_SOURCE_DIR}/src/query
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${IODBC_INCLUDE_DIRS}
	${GSL_INCLUDE_DIR}
)

ADD_EXECUTABLE(IndefiniteRuleTest IndefiniteRuleTest.cc)

TARGET_LINK_LIBRARIES(IndefinitePLNFormulas
	atomspace
	xml
	util
	query
	schemes
	wsd
	persist
	${IODBC_LIBRARIES}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
	${EXPAT_LIBRARIES}
	${GUILE_LIBRARIES}
	${GSL_LIBRARIES}
)

TARGET_LINK_LIBRARIES(IndefiniteRuleTest IndefinitePLNFormulas)
