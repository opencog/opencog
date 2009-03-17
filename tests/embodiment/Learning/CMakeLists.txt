ADD_SUBDIRECTORY (behavior)
ADD_SUBDIRECTORY (LearningServerMessages)
#ADD_SUBDIRECTORY (FitnessEstimator)
ADD_SUBDIRECTORY (Filter)
ADD_SUBDIRECTORY (NoSpaceLife)
ADD_SUBDIRECTORY (hillclimbing)

INCLUDE_DIRECTORIES (
	${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
	${PETAVERSE_SOURCE_DIR}/src/Control/PerceptionActionInterface
	${PETAVERSE_SOURCE_DIR}/test/Control/PerceptionActionInterface
	${PETAVERSE_SOURCE_DIR}/src/Control/Predavese
	${PETAVERSE_SOURCE_DIR}/src/Control
	${PETAVERSE_SOURCE_DIR}/src/Spatial
        ${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
	${PETAVERSE_SOURCE_DIR}/src/Learning
	${PETAVERSE_SOURCE_DIR}/test/Learning/behavior
	${PETAVERSE_SOURCE_DIR}/src/Learning/behavior
	${PETAVERSE_SOURCE_DIR}/include/xerces
)

LINK_DIRECTORIES (
	${PETAVERSE_BINARY_DIR}/src/AtomSpaceExtensions
	${PETAVERSE_BINARY_DIR}/src/Control/MessagingSystem
	${PETAVERSE_BINARY_DIR}/src/Control/OperationalPetController
	${PETAVERSE_BINARY_DIR}/src/Control/PerceptionActionInterface
	${PETAVERSE_SOURCE_DIR}/lib/linux/xerces
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
