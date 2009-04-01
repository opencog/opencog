ADD_SUBDIRECTORY (behavior)
ADD_SUBDIRECTORY (LearningServerMessages)
ADD_SUBDIRECTORY (FitnessEstimator)
ADD_SUBDIRECTORY (Filter)
ADD_SUBDIRECTORY (NoSpaceLife)
ADD_SUBDIRECTORY (PetaverseHC)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/test/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/test/embodiment/Learning/behavior
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/OperationalPetController
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_BINARY_DIR}/opencog/util
)

#ADD_CXXTEST(EdaUTest)
#TARGET_LINK_LIBRARIES(EdaUTest
#	eda
#   ${LADSUTIL_LIBRARY}
#)

#ADD_CXXTEST(KnobTurningUTest)
#TARGET_LINK_LIBRARIES(KnobTurningUTest
#	${MOSES_LIBRARY}
#    ${COMBOREDUCT_LIBRARY}
#    ${LADSUTIL_LIBRARY}
#)

ADD_CXXTEST(SetopsUTest)
TARGET_LINK_LIBRARIES(SetopsUTest
    util
)

#ADD_CXXTEST(BehaviorUTest)
#TARGET_LINK_LIBRARIES(BehaviorUTest
#	 behavior
#    PAI
#    Predavese
#    Control
#    ${OPENCOG_LIBARIES}
#    ${LADSUTIL_LIBRARY}
#    xerces-c
#    ${Boost_DATE_TIME_LIBRARY}
#)


#ADD_CXXTEST(TopologicalSortUTest)
#TARGET_LINK_LIBRARIES(TopologicalSortUTest
#	${LADSUTIL_LIBRARY}
#)
