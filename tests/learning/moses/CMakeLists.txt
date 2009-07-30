ADD_CXXTEST(KnobTurningUTest)
TARGET_LINK_LIBRARIES(KnobTurningUTest
    moses
    comboreduct
    util
)

ADD_CXXTEST(NeighborSamplingUTest)
TARGET_LINK_LIBRARIES(NeighborSamplingUTest
    moses
    comboreduct
    util
)

ADD_CXXTEST(FieldSetUTest)
TARGET_LINK_LIBRARIES(FieldSetUTest
    moses
    comboreduct
    util
)
