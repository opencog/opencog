ADD_LIBRARY(xpatternminer SHARED
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
	DESTINATION "include/opencog/learning/xpattern-miner"
)

IF (HAVE_GUILE)
  ADD_LIBRARY(guile-xpatternminer SHARED
    XPatternMinerSCM
  )

  TARGET_LINK_LIBRARIES(guile-xpatternminer
    xpatternminer
    ${GUILE_LIBRARIES}
  )

  INSTALL (TARGETS guile-xpatternminer DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

  ADD_GUILE_MODULE (FILES
    xpattern-miner.scm
    xpattern-miner-utils.scm
    MODULE_DESTINATION "${DATADIR}/scm/opencog/xpattern-miner"
  )

  ADD_SUBDIRECTORY (rules)
ENDIF (HAVE_GUILE)
