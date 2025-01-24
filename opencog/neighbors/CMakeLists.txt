ADD_LIBRARY (neighbors
	GetPredicates.cc
	Neighbors.cc
)

TARGET_LINK_LIBRARIES(neighbors
	atombase
	${COGUTIL_LIBRARY}
)

INSTALL (FILES
	GetPredicates.h
	FollowLink.h
	ForeachChaseLink.h
	Neighbors.h
	DESTINATION "include/opencog/neighbors"
)
