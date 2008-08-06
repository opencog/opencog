SET(CMAKE_BUILD_TYPE Debug)
IF (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-deprecated -Wno-unused-variable")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

ADD_LIBRARY (pln STATIC
    PLNFormulas/PLNFormulas.cc
    PLNFormulas/PLNFormulasIndefinite.cc
    PLNFormulas/deductionLookupTable.cc
    BackInferenceTreeNode.cc
	#BackwardInferenceTask.cc
    Ptlatom.cc
    Grim.cc
    XMLNode.cc
    PLNUtils.cc
    #BOASimulatorAgent.cc
    PLNShell.cc
    Tester.cc
	PLNRules/Rule.cc
    PLNRules/RuleApp.cc
    PLNRules/RuleProvider.cc
	PLNRules/RuleFunctions.cc
	PLNRules/NOTRules/NotEvaluatorRule.cc
	PLNRules/ORRules/ORPartitionRule.cc
	PLNRules/ORRules/ORRule.cc
	PLNRules/AuxiliarRules/SubsetEvalRule.cc
	PLNRules/AuxiliarRules/CrispTheoremRule.cc
	PLNRules/AuxiliarRules/LookupRule.cc
	PLNRules/AuxiliarRules/ScholemFunctionProductionRule.cc
	PLNRules/AuxiliarRules/UnorderedLinkPermutationRule.cc
	PLNRules/UnificationRules/BaseCrispUnificationRule.cc
	PLNRules/UnificationRules/CustomCrispUnificationRule.cc
	PLNRules/UnificationRules/CustomCrispUnificationRuleComposer.cc
	PLNRules/UnificationRules/StrictCrispUnificationRule.cc
	PLNRules/ANDRules/ANDRuleArityFree.cc
	PLNRules/ANDRules/ANDRule.cc
	PLNRules/ANDRules/ANDPartitionRule.cc
	PLNRules/ANDRules/ANDSubstRule.cc
	PLNRules/ANDRules/ANDBreakdownRule.cc
	PLNRules/ImplicationRules/ImplicationTailExpansionRule.cc
	PLNRules/ImplicationRules/ImplicationBreakdownRule.cc
	PLNRules/ImplicationRules/StrictImplicationBreakdownRule.cc
	PLNRules/ImplicationRules/ImplicationConstructionRule.cc
	PLNRules/InferenceRules/HypothesisRule.cc
	PLNRules/ConversionRules/Equi2ImpRule.cc
	PLNRules/SimSubstRules/SimSubstRule2.cc
	PLNRules/SimSubstRules/SimSubstRule1.cc
	AtomTableWrapper.cc
    XMLNodeLoader.cc
    # for later:
    #CrispUnificationRule.cc
    # convert to OpenCog specific class:
    NMPrinter.cc
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

ADD_EXECUTABLE(PLNFormulaUTest PLNFormulas/PLNFormulaUTest.cc)
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
TARGET_LINK_LIBRARIES(PLNShell pln
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

