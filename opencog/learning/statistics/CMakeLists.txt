ADD_LIBRARY (statistics SHARED
	DataProvider
	Probability
	Entropy
	InteractionInformation
)

TARGET_LINK_LIBRARIES (statistics
	${COGUTIL_LIBRARY}
)

INSTALL (TARGETS statistics
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	DataProvider.h
	Probability.h
	Entropy.h
	InteractionInformation.h
	DESTINATION "include/${PROJECT_NAME}/learning/statistics"
)

# Install library
IF (WIN32)
	INSTALL(TARGETS statistics DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
	INSTALL(TARGETS statistics LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)


