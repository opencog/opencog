INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/Learning/NoSpaceLife
	${OPENCOG_SOURCE_DIR}/src/Learning/behavior
	${OPENCOG_SOURCE_DIR}/src/Learning
	${OPENCOG_SOURCE_DIR}/src/PetComboVocabulary
	${OPENCOG_SOURCE_DIR}/src/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/src/Spatial
	${OPENCOG_SOURCE_DIR}/src/Spatial/Math
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/PetComboVocabulary
	${OPENCOG_BINARY_DIR}/src/Learning/behavior
	${OPENCOG_BINARY_DIR}/src/Learning/NoSpaceLife
	${OPENCOG_BINARY_DIR}/src/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/lib/linux/xerces
)

ADD_CXXTEST(NoSpaceLifeUTest)
TARGET_LINK_LIBRARIES(NoSpaceLifeUTest
	behavior
	ImaginaryLife
	xerces-c
        comboreduct
        util
	${OPENCOG_LIBRARIES}
)

