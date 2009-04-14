ADD_CXXTEST(NoSpaceLifeUTest)
TARGET_LINK_LIBRARIES(NoSpaceLifeUTest
	behavior
	ImaginaryLife
        comboreduct
        util
	${OPENCOG_LIBRARIES}
)

