ADD_LIBRARY (FitnessEstimator SHARED
	NoSpaceLifeFitnessEstimator
	DistortedComboSize
	SizePenalty
)


TARGET_LINK_LIBRARIES (FitnessEstimator
	behavior
	WorldWrapper
	ImaginaryLife
	Procedure
	AtomSpaceExtensions
	comboreduct
	cogutil
)
