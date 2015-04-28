
ADD_LIBRARY (savable SHARED
	CoreUtils.cc
	SavingLoading.cc
	SpaceServerSavable.cc
	TimeServerSavable.cc
	TemporalTableFile.cc
)

ADD_DEPENDENCIES(savable opencog_atom_types)

INSTALL (FILES
	SavableRepository.h
	SavingLoading.h
	DESTINATION "include/${PROJECT_NAME}/persist/file"
)
