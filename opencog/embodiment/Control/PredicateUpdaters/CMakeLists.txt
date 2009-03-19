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
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/include/xerces
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog
)


LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/util
)

SET(CMAKE_BUILD_TYPE Debug)

TARGET_LINK_LIBRARIES (PredicateUpdaters
	${OPENCOG_LIBRARIES}
)

