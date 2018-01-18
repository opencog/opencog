ADD_LIBRARY(xpatternminer
  XPatternMiner
  HandleTree
  Valuations
)

TARGET_LINK_LIBRARIES(xpatternminer
  ${COGUTIL_LIBRARY}
  ${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS xpatternminer DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

INSTALL (FILES
	XPatternMiner.h
	HandleTree.h
    Valuations.h
	DESTINATION "include/opencog/learning/xpattern-miner/c++-based"
)
