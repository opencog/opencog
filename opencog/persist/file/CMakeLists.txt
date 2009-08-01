
ADD_LIBRARY (savable SHARED
	CoreUtils.cc
	CompositeRenumber.cc
	SavingLoading.cc
	SpaceServerSavable.cc
	TimeServerSavable.cc
	TemporalTableFile.cc
)

INSTALL (FILES
	SavableRepository.h
	SavingLoading.h
	DESTINATION "include/opencog/persist/file"
)
