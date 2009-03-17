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
	${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
	${PETAVERSE_SOURCE_DIR}/src/Control/PerceptionActionInterface
	${PETAVERSE_SOURCE_DIR}/include/xerces
	${PETAVERSE_SOURCE_DIR}/src/Spatial
	${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
	${PETAVERSE_SOURCE_DIR}/src/Control
	${PETAVERSE_SOURCE_DIR}/src/Control/Predavese
)


LINK_DIRECTORIES (
	${PETAVERSE_BINARY_DIR}/src/AtomSpaceExtensions
)

SET(CMAKE_BUILD_TYPE Debug)

#TARGET_LINK_LIBRARIES (PredicateUpdaters
#	${OPENCOG_LIBRARIES}
#)

