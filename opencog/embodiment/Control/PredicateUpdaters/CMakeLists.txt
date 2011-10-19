ADD_LIBRARY (PredicateUpdaters 
	PredicatesUpdater.cc
	BasicPredicateUpdater.cc 
	IsSmallPredicateUpdater.cc 
	IsMovablePredicateUpdater.cc 
	IsPickupablePredicateUpdater.cc 
	IsNoisyPredicateUpdater.cc
	IsPeePlacePredicateUpdater.cc 
	IsPooPlacePredicateUpdater.cc
    SpatialPredicateUpdater.cc 
	PetPsychePredicatesUpdater.cc
)

SET(CMAKE_BUILD_TYPE Debug)

TARGET_LINK_LIBRARIES (PredicateUpdaters
    AtomSpaceExtensions
	${PROJECT_LIBRARIES}
)

