SET(CMAKE_BUILD_TYPE Debug)
IF (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-deprecated -Wno-unused-variable")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

ADD_LIBRARY (pln SHARED
    PLNModule.cc
	AtomSpaceWrapper.cc
    BackInferenceTreeNode.cc
    PLNatom.cc
    PLNUtils.cc
#    PLNShell.cc
    PLNGlobals.cc
#    Tester.cc
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
#    utils/XMLNode.cc
#    utils/XMLNodeLoader.cc
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
    ${OPENCOG_SOURCE_DIR}/src/adaptors/tulip
    ${OPENCOG_SOURCE_DIR}/src/nlp/wsd
    ${OPENCOG_SOURCE_DIR}/src/persist
    ${OPENCOG_SOURCE_DIR}/src/query
    ${OPENCOG_SOURCE_DIR}/src/reasoning
    ${CSOCKETS_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
    ${IODBC_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
)

#ADD_EXECUTABLE(plnshell PLNShell.cc)

TARGET_LINK_LIBRARIES(pln
    atomspace
#    server
    xml
    util
    #query
#    smob #scheme
#    adaptor-tulip
    ${IODBC_LIBRARIES}
    ${CSOCKETS_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    ${EXPAT_LIBRARIES}
#    ${GUILE_LIBRARIES}
    ${GSL_LIBRARIES}
    ${Boost_SIGNALS_LIBRARY}
)

#TARGET_LINK_LIBRARIES(plnshell pln
#    #atomspace
#    server
#    xml
#    util
#    #query
#    smob #scheme
#    #wsd
#    persist
#   adaptor-tulip
#    ${IODBC_LIBRARIES}
#    ${CSOCKETS_LIBRARIES}
#    ${OPENSSL_LIBRARIES}
#    ${EXPAT_LIBRARIES}
#    ${GUILE_LIBRARIES}
#    ${GSL_LIBRARIES}
#    ${Boost_SIGNALS_LIBRARY}
#)

INSTALL (TARGETS pln
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
    PLNUtils.h
    PLNModule.h
    AtomLookupProvider.h
    PLNatom.h
    PLN.h
    BackInferenceTreeNode.h
    TestTargets.h
    rules/Rule.h
    rules/implication/ImplicationRedundantExpansionRule.h
    rules/implication/ImplicationBreakdownRule.h
    rules/implication/ImplicationConstructionRule.h
    rules/implication/StrictImplicationBreakdownRule.h
    rules/implication/ImplicationTailExpansionRule.h
    rules/simsubst/SimSubstRule2.h
    rules/simsubst/SimSubstRule1.h
    rules/or/ORRule.h
    rules/or/OR2ANDRule.h
    rules/or/ORPartitionRule.h
    rules/RuleApp.h
    rules/inference/DeductionRule.h
    rules/inference/HypothesisRule.h
    rules/inference/InversionRule.h
    rules/inference/RevisionRule.h
    rules/inference/QuantifierRule.h
    rules/not/NotEvaluatorRule.h
    rules/not/NotEliminationRule.h
    rules/auxiliary/LookupRule.h
    rules/auxiliary/VarInstantiationRule.h
    rules/auxiliary/ScholemFunctionProductionRule.h
    rules/auxiliary/SubsetEvalRule.h
    rules/auxiliary/PrintRule.h
    rules/auxiliary/UnorderedLinkPermutationRule.h
    rules/auxiliary/CrispTheoremRule.h
    rules/convert/Ext2ExtRule.h
    rules/convert/Inh2EvalRule.h
    rules/convert/Sim2InhRule.h
    rules/convert/Inh2SimRule.h
    rules/convert/Exist2ForAllRule.h
    rules/convert/Eval2MemRule.h
    rules/convert/Link2LinkRule.h
    rules/convert/Equi2ImpRule.h
    rules/convert/Inclusion2EvalRule.h
    rules/RuleProvider.h
    rules/RuleFunctions.h
    rules/GenericRule.h
    rules/unify/CrispUnificationRule.h
    rules/unify/CustomCrispUnificationRuleComposer.h
    rules/unify/CustomCrispUnificationRule.h
    rules/unify/BaseCrispUnificationRule.h
    rules/unify/StrictCrispUnificationRule.h
    rules/Rules.h
    rules/and/AND2ORRule.h
    rules/and/ANDBreakdownRule.h
    rules/and/ANDSubstRule.h
    rules/and/ANDRuleArityFree.h
    rules/and/ANDRuleSimple.h
    rules/and/ANDRule.h
    rules/and/ANDPartitionRule.h
    AtomSpaceWrapper.h
#    utils/XMLNodeLoader.h
    utils/fim.h
    utils/Singleton.h
    utils/NMPrinter.h
#    utils/XMLNode.h
    ForwardChainer.h
    formulas/VectorLookupTable.h
    formulas/FormulasIndefinite.h
    formulas/LookupTable.h
    formulas/Formula.h
    formulas/DeductionLookupTable.h
    formulas/Formulas.h
    iAtomSpaceWrapper.h
    PLNEvaluator.h
#    PLNShell.h
	DESTINATION "include/opencog/reasoning/pln"
)
