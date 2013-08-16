#SET(CMAKE_BUILD_TYPE Debug)
IF (CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-deprecated -Wno-unused-variable")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)


OPENCOG_ADD_ATOM_TYPES(atom_types.script 
   atom_types.h
   atom_types.definitions
   atom_types.inheritance
   pln_types.scm)

ADD_CUSTOM_TARGET(pln_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (pln-types SHARED
   atom_types
   PLNTypes
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(pln-types pln_atom_types)

ADD_LIBRARY (pln SHARED
	PLNModule.cc
	AtomSpaceWrapper.cc
	BackInferenceTreeNode.cc
	InferenceCache.cc
	PLNatom.cc
	PLNUtils.cc
#	PLNShell.cc
	PLNGlobals.cc
#	Tester.cc
	Testing.cc
	ForwardChainer.cc
	BackChainingAgent
	ForwardChainingAgent
# Formulas:
	formulas/Formulas.cc
	formulas/FormulasIndefinite.cc
	formulas/DeductionLookupTable.cc
# Rules:
	rules/Rule.cc
	rules/RuleApp.cc
	rules/RuleProvider.cc
	rules/RuleFunctions.cc
	rules/not/NotRule.cc
	rules/or/OrPartitionRule.cc
	rules/or/OrRule.cc
	rules/auxiliary/SubsetEvalRule.cc
	rules/auxiliary/IntensionalInheritanceRule
	rules/auxiliary/CrispTheoremRule.cc
	rules/auxiliary/LookupRule.cc
	rules/auxiliary/ScholemFunctionProductionRule.cc
	rules/auxiliary/UnorderedLinkPermutationRule.cc
	rules/and/AndRuleArityFree.cc
	rules/and/AndRule.cc
	rules/and/AndPartitionRule.cc
	rules/and/AndSubstRule.cc
	rules/and/AndBreakdownRule.cc
	rules/implication/ImplicationTailExpansionRule.cc
	rules/implication/ImplicationBreakdownRule.cc
	rules/implication/StrictImplicationBreakdownRule.cc
	rules/implication/ImplicationConstructionRule.cc
	rules/inference/HypothesisRule.cc
	rules/convert/Equi2ImpRule.cc
	rules/context/ContextualizerRule.cc
	rules/context/DecontextualizerRule.cc
	rules/context/ContextFreeToSensitiveRule.cc
# Utils
	utils/Grim.cc
	utils/NMPrinter.cc
	utils/mva.cc
	# for later:
	#CrispUnificationRule.cc
	# convert to OpenCog specific class:
	#TimeStamp.cc
	#spacetime.cc
	ASSOC
)

# logic_wrapper is in the cython directory
ADD_DEPENDENCIES(pln logic_wrapper)
ADD_DEPENDENCIES(pln opencog_atom_types)
ADD_DEPENDENCIES(pln pln_atom_types)

INCLUDE_DIRECTORIES (
	# For the Python PLN binding
	${PYTHON_INCLUDE_PATH}
	${CMAKE_BINARY_DIR}
)

SET(PLN_LINK_LIBS
	atomspace
	server
	cogutil
	pln-types
	#${Boost_SIGNALS_LIBRARY}
)

IF (HAVE_CYTHON)
	SET(PLN_LINK_LIBS
#		logic_wrapper
		PythonModule
		${PYTHON_LIBRARIES}
	${PLN_LINK_LIBS})
ENDIF (HAVE_CYTHON)

IF (HAVE_GUILE)
	SET(PLN_LINK_LIBS smob ${PLN_LINK_LIBS})
ENDIF (HAVE_GUILE)

IF (HAVE_EXPAT)
	SET(PLN_LINK_LIBS xml ${PLN_LINK_LIBS})
ENDIF (HAVE_EXPAT)

IF (XMLRPC_FOUND AND CURL_FOUND) # The code which uses ubigraph will be compiled
	SET(PLN_LINK_LIBS ubigraph ${PLN_LINK_LIBS})
ENDIF (XMLRPC_FOUND AND CURL_FOUND)

TARGET_LINK_LIBRARIES(pln ${PLN_LINK_LIBS})

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
	InferenceCache.h
	TestTargets.h
	BackChainingAgent.h
	ForwardChainingAgent.h
	rules/Rule.h
	rules/implication/ImplicationRedundantExpansionRule.h
	rules/implication/ImplicationBreakdownRule.h
	rules/implication/ImplicationConstructionRule.h
	rules/implication/StrictImplicationBreakdownRule.h
	rules/implication/ImplicationTailExpansionRule.h
	rules/substitution/BaseSubstRule.h
	rules/substitution/InheritanceSubstRule.h
	rules/substitution/SimilaritySubstRule.h
	rules/or/OrRule.h
	rules/or/Or2AndRule.h
	rules/or/OrPartitionRule.h
	rules/RuleApp.h
	rules/inference/DeductionRule.h
	rules/inference/HypothesisRule.h
	rules/inference/InversionRule.h
	rules/inference/RevisionRule.h
	rules/inference/QuantifierRule.h
	rules/not/NotRule.h
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
	rules/instantiation/BaseInstantiationRule.h
	rules/instantiation/ForAllInstantiationRule.h
	rules/instantiation/AverageInstantiationRule.h
	rules/Rules.h
	rules/and/And2OrRule.h
	rules/and/AndBreakdownRule.h
	rules/and/AndSubstRule.h
	rules/and/AndRuleArityFree.h
	rules/and/AndRuleSimple.h
	rules/and/AndRule.h
	rules/and/AndPartitionRule.h
	rules/context/ContextualizerRule.h
	rules/context/DecontextualizerRule.h
	rules/context/ContextFreeToSensitiveRule.h
	AtomSpaceWrapper.h
#	utils/XMLNodeLoader.h
	utils/fim.h
	utils/Singleton.h
	utils/NMPrinter.h
#	utils/XMLNode.h
	ForwardChainer.h
	formulas/VectorLookupTable.h
	formulas/FormulasIndefinite.h
	formulas/LookupTable.h
	formulas/Formula.h
	formulas/DeductionLookupTable.h
	formulas/Formulas.h
	iAtomSpaceWrapper.h
	PLNEvaluator.h
#	PLNShell.h
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h

	DESTINATION "include/${PROJECT_NAME}/reasoning/pln"
)
