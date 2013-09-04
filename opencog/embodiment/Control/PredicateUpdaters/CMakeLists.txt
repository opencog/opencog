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

ADD_DEPENDENCIES(PredicateUpdaters opencog_atom_types)
ADD_DEPENDENCIES(PredicateUpdaters spacetime_types)
ADD_DEPENDENCIES(PredicateUpdaters nlp_atom_types)
ADD_DEPENDENCIES(PredicateUpdaters embodiment_atom_types)


SET(CMAKE_BUILD_TYPE Debug)

TARGET_LINK_LIBRARIES (PredicateUpdaters
    AtomSpaceExtensions
	${PROJECT_LIBRARIES}
)

