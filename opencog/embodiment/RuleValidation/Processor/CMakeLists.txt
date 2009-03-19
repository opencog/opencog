INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog
    ${OPENCOG_SOURCE_DIR}/include/lua
    ${OPENCOG_SOURCE_DIR}/include/luabind
    ${OPENCOG_SOURCE_DIR}/include/xerces
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_SOURCE_DIR}/lib/linux/lua
    ${OPENCOG_SOURCE_DIR}/lib/linux/luabind
    ${OPENCOG_SOURCE_DIR}/lib/linux/xerces
)

ADD_LIBRARY(Processor
    RuleProcessor
)

TARGET_LINK_LIBRARIES(Processor
    lua5.1
    luabind
    xerces-c
    VirtualWorldData
    comboreduct
    util
)

#ADD_EXECUTABLE (ruleProcessor ProcessorExec.cc)
#TARGET_LINK_LIBRARIES (ruleProcessor
#    Processor
#    Procedure
#    ImaginaryLife
#)

