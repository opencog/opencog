ADD_CXXTEST(KnobTurningUTest)
TARGET_LINK_LIBRARIES(KnobTurningUTest
    moses
    comboreduct
    util
)

# requires boost::math; don't build if not found.
IF(Boost_MATH_C99_FOUND)
	ADD_CXXTEST(NeighborSamplingUTest)
	TARGET_LINK_LIBRARIES(NeighborSamplingUTest
    	moses
    	comboreduct
    	util
	)
ENDIF(Boost_MATH_C99_FOUND)


ADD_CXXTEST(FieldSetUTest)
TARGET_LINK_LIBRARIES(FieldSetUTest
    moses
    comboreduct
    util
)

ADD_CXXTEST(representationUTest)
TARGET_LINK_LIBRARIES(representationUTest
    moses
    comboreduct
    util
)

ADD_CXXTEST(MOSESUTest)
TARGET_LINK_LIBRARIES(MOSESUTest
    moses
    comboreduct
    util        
)
