ADD_LIBRARY(xpatternminer
  XPatternMiner
  HandleTree
)

TARGET_LINK_LIBRARIES(xpatternminer
  ${COGUTIL_LIBRARY}
  ${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS xpatternminer DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

INSTALL (FILES
	XPatternMiner.h
	HandleTree
	DESTINATION "include/opencog/learning/experimental-pattern-miner/c++-based"
)
