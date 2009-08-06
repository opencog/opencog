ADD_LIBRARY (FitnessEstimator
	NoSpaceLifeFitnessEstimator
	DistortedComboSize
	SizePenalty
)

INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
)

TARGET_LINK_LIBRARIES (FitnessEstimator
	util
	${XERCESC_LIBRARIES}
	comboreduct
	behavior
	WorldWrapper
	AtomSpaceExtensions
	ImaginaryLife
)
