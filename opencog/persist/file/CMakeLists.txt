
ADD_LIBRARY (savable SHARED
	CoreUtils.cc
	CompositeRenumber.cc
	SavingLoading.cc
	SpaceServerSavable.cc
	TimeServerSavable.cc
	TemporalTableFile.cc
)

#IF(APPLE)
#    TARGET_LINK_LIBRARIES (savable
##        atomspace
#    )
#ENDIF(APPLE)

INSTALL (FILES
	SavableRepository.h
	SavingLoading.h
	DESTINATION "include/opencog/persist/file"
)
