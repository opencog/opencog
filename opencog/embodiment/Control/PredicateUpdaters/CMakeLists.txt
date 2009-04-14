ADD_LIBRARY (PredicateUpdaters 
	PredicatesUpdater.cc
	BasicPredicateUpdater.cc 
	IsSmallPredicateUpdater.cc 
	IsMovablePredicateUpdater.cc 
	IsPickupablePredicateUpdater.cc 
	IsNoisyPredicateUpdater.cc
	IsPeePlacePredicateUpdater.cc 
	IsPooPlacePredicateUpdater.cc
	NearPredicateUpdater.cc 
	PetPsychePredicatesUpdater.cc
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog
)


SET(CMAKE_BUILD_TYPE Debug)

TARGET_LINK_LIBRARIES (PredicateUpdaters
	${OPENCOG_LIBRARIES}
)

