INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
)

ADD_LIBRARY(Processor
    RuleProcessor
)

TARGET_LINK_LIBRARIES(Processor
    ${XERCESC_LIBRARIES}
    VirtualWorldData
    Procedure
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

