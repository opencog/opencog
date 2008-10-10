SET(CMAKE_BUILD_TYPE Debug)
IF (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-deprecated -Wno-unused-variable")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

ADD_LIBRARY (pln STATIC
	AtomTableWrapper.cc
    BackInferenceTreeNode.cc
    PLNatom.cc
    PLNUtils.cc
    PLNShell.cc
    Tester.cc
    ForwardChainer.cc
# Formulas:
    formulas/Formulas.cc
    formulas/FormulasIndefinite.cc
    formulas/DeductionLookupTable.cc
# Rules:
	rules/Rule.cc
    rules/RuleApp.cc
    rules/RuleProvider.cc
	rules/RuleFunctions.cc
	rules/not/NotEvaluatorRule.cc
	rules/or/ORPartitionRule.cc
	rules/or/ORRule.cc
	rules/auxiliary/SubsetEvalRule.cc
	rules/auxiliary/CrispTheoremRule.cc
	rules/auxiliary/LookupRule.cc
	rules/auxiliary/ScholemFunctionProductionRule.cc
	rules/auxiliary/UnorderedLinkPermutationRule.cc
	rules/unify/BaseCrispUnificationRule.cc
	rules/unify/CustomCrispUnificationRule.cc
	rules/unify/CustomCrispUnificationRuleComposer.cc
	rules/unify/StrictCrispUnificationRule.cc
	rules/and/ANDRuleArityFree.cc
	rules/and/ANDRule.cc
	rules/and/ANDPartitionRule.cc
	rules/and/ANDSubstRule.cc
	rules/and/ANDBreakdownRule.cc
	rules/implication/ImplicationTailExpansionRule.cc
	rules/implication/ImplicationBreakdownRule.cc
	rules/implication/StrictImplicationBreakdownRule.cc
	rules/implication/ImplicationConstructionRule.cc
	rules/inference/HypothesisRule.cc
	rules/convert/Equi2ImpRule.cc
	rules/simsubst/SimSubstRule2.cc
	rules/simsubst/SimSubstRule1.cc
# Utils
    utils/Grim.cc
    utils/XMLNode.cc
    utils/XMLNodeLoader.cc
    utils/NMPrinter.cc
    # for later:
    #CrispUnificationRule.cc
    # convert to OpenCog specific class:
	#TimeStamp.cc
	#spacetime.cc
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

ADD_EXECUTABLE(plnshell PLNShell.cc)
ADD_EXECUTABLE(indeftest formulas/IndefiniteRuleTest.cc)

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

TARGET_LINK_LIBRARIES(plnshell pln
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

TARGET_LINK_LIBRARIES(indeftest pln
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
