SET(CMAKE_BUILD_TYPE Debug)

ADD_LIBRARY (pln STATIC
	deductionLookupTable.cc
	AtomTableWrapper.cc
	BackInferenceTree.cc
	BackInferenceTreeNode.cc
	BackwardInferenceTask.cc
	CrispUnificationRule.cc
	IndefinitePLNFormulas.cc
	PLNUtils.cc
	PTLFormulaeKit.cc
	RuleProvider.cc
	Rules.cc
	Tester.cc
	XMLNode.cc
	XMLNodeLoader.cc
	PLNShell.cc
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
	${OPENCOG_SOURCE_DIR}/src/reasoning
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${IODBC_INCLUDE_DIRS}
	${GSL_INCLUDE_DIR}
)

ADD_EXECUTABLE(PLNFormulaUTest PLNFormulaUTest.cc)
ADD_EXECUTABLE(PLNShell PLNShell.cc)

TARGET_LINK_LIBRARIES(pln
	atomspace
	server
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

TARGET_LINK_LIBRARIES(PLNFormulaUTest pln)
TARGET_LINK_LIBRARIES(PLNShell pln)
