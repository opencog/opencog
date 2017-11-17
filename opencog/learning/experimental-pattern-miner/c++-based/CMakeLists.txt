ADD_LIBRARY(xpatternminer
  XPatternMiner
)

TARGET_LINK_LIBRARIES(xpatternminer
  ${COGUTIL_LIBRARY}
  ${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS xpatternminer DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

INSTALL (FILES
	XPatternMiner.h
	DESTINATION "include/opencog/learning/experimental-pattern-miner/c++-based"
)
