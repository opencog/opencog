
ADD_LIBRARY (savable SHARED
	CoreUtils.cc
	CompositeRenumber.cc
	SavingLoading.cc
	SpaceServerSavable.cc
	TimeServerSavable.cc
	TemporalTableFile.cc
)
TARGET_LINK_LIBRARIES(savable space_server)

ADD_DEPENDENCIES(savable opencog_atom_types)

INSTALL (FILES
	SavableRepository.h
	SavingLoading.h
	DESTINATION "include/${PROJECT_NAME}/persist/file"
)
