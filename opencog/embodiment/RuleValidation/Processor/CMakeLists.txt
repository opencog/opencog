INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${LUABIND_INCLUDE_DIR}
)

ADD_LIBRARY(Processor
    RuleProcessor
)

TARGET_LINK_LIBRARIES(Processor
    ${LUA_LIBRARIES}
    ${LUABIND_LIBRARIES}
    ${XERCESC_LIBRARIES}
    VirtualWorldData
    comboreduct
    util
)

# These libaries will be linked for ruleProcessor
SET(RULE_LIBS
    Processor
    Procedure
    ImaginaryLife
    util
)

IF (GUILE_FOUND)
   SET(RULE_LIBS smob ${RULE_LIBS})
ENDIF (GUILE_FOUND)

ADD_EXECUTABLE (ruleProcessor ProcessorExec.cc)
TARGET_LINK_LIBRARIES (ruleProcessor ${RULE_LIBS})

