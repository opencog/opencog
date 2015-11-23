ADD_LIBRARY(planning SHARED
    Action
    ActionSelector
)

INSTALL (TARGETS planning DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
