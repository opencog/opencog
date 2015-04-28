ADD_LIBRARY(ImaginaryLife SHARED
	NoSpaceLife
)

TARGET_LINK_LIBRARIES(ImaginaryLife
	behavior
	AtomSpaceExtensions
	comboreduct
	cogutil
)
