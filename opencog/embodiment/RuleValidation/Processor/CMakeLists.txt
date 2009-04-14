INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
    ${LUA51_INCLUDE_DIR}
    ${LUABIND_INCLUDE_DIR}
)

ADD_LIBRARY(Processor
    RuleProcessor
)

TARGET_LINK_LIBRARIES(Processor
    ${LUA51_LIBRARIES}
    ${LUABIND_LIBRARIES}
    ${XERCESC_LIBRARIES}
    VirtualWorldData
    comboreduct
    util
)

ADD_EXECUTABLE (ruleProcessor ProcessorExec.cc)
TARGET_LINK_LIBRARIES (ruleProcessor
    Processor
    Procedure
    ImaginaryLife
)

