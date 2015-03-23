ADD_LIBRARY (oac SHARED
	Pet
	Plaza
	OAC
	PetMessageSender
	PVPActionPlanSender
	ProcedureInterpreterAgent
	ActionSelectionAgent
	EntityExperienceAgent
	PsiModulatorUpdaterAgent
	PsiDemandUpdaterAgent
	PsiActionSelectionAgent
	PsiRelationUpdaterAgent
	PsiFeelingUpdaterAgent
	EventDetectionAgent
	PsiRuleUtil
	StimulusUpdaterAgent
	SleepAgent
	MockOpcHCTest
	HCTestAgent
	BaseAgentModeHandler
	DefaultAgentModeHandler
	LearningAgentModeHandler
	PlanningHeaderFiles
	Strips
	OCPlanner
	Inquery
	OCPlanningAgent
	PatternMiningAgent
)

ADD_DEPENDENCIES(oac opencog_atom_types)
ADD_DEPENDENCIES(oac spacetime_types)
ADD_DEPENDENCIES(oac nlp_atom_types)
ADD_DEPENDENCIES(oac embodiment_atom_types)

INCLUDE_DIRECTORIES (
	${PYTHON_INCLUDE_PATH}
	${CMAKE_SOURCE_DIR}
	${CMAKE_BINARY_DIR}
)

TARGET_LINK_LIBRARIES (oac
	PredicateUpdaters
	PAI
	Language
	behavior
	MessagingSystem
	Procedure
	savable
	attention
	SpaceMap
	json_spirit
	PatternMiner
)

IF (HAVE_CYTHON)
	TARGET_LINK_LIBRARIES (oac
		PythonEval
		PythonModule
		${PYTHON_LIBRARIES}
	)
ENDIF (HAVE_CYTHON)	


IF (HAVE_ZMQ)
	TARGET_LINK_LIBRARIES (oac ${ZMQ_LIBRARIES})
ENDIF (HAVE_ZMQ)


ADD_EXECUTABLE (OACTester OACTester.cc)
TARGET_LINK_LIBRARIES (OACTester
	oac
	comboreduct
	comboant
	AvatarComboVocabulary
	AtomSpaceExtensions
	ImaginaryLife
	${Boost_THREAD_LIBRARY}
)

ADD_EXECUTABLE (oac-exec OACExecutable.cc)
TARGET_LINK_LIBRARIES (oac-exec
	oac
	comboreduct
	AvatarComboVocabulary
	ImaginaryLife
	${Boost_THREAD_LIBRARY} 
	${ZMQ_LIBRARIES}
)
SET_TARGET_PROPERTIES(oac-exec PROPERTIES OUTPUT_NAME "oac")

# ------------------------------------------------------

ADD_EXECUTABLE (opcHcTester MockOpcHCTestExec.cc)
TARGET_LINK_LIBRARIES (opcHcTester
	oac
	comboreduct
	AvatarComboVocabulary
	ImaginaryLife
)
IF (HAVE_PROTOBUF)
	TARGET_LINK_LIBRARIES (opcHcTester LearningServerMessages ${PROTOBUF_LIBRARY})
ENDIF (HAVE_PROTOBUF)

