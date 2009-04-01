INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/NoSpaceLife
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/NoSpaceLife
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/util
	${OPENCOG_BINARY_DIR}/opencog/comboreduct
)

ADD_CXXTEST(NoSpaceLifeUTest)
TARGET_LINK_LIBRARIES(NoSpaceLifeUTest
	behavior
	ImaginaryLife
        comboreduct
        util
	${OPENCOG_LIBRARIES}
)

