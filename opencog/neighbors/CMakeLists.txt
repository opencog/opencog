ADD_LIBRARY (neighbors
	Neighbors
)

TARGET_LINK_LIBRARIES(neighbors
	atombase
	${COGUTIL_LIBRARY}
)

INSTALL (FILES
	FollowLink.h
	ForeachChaseLink.h
	Neighbors.h
	DESTINATION "include/opencog/neighbors"
)
