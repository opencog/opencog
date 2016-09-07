IF(CMAKE_COMPILER_IS_GNUCXX)
	# Best compromise between compile time and performance:
	# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -finline-functions")

	# Fastest:
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -fno-inline")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ADD_LIBRARY(SpaceMap SHARED
	LocalSpaceMap2DUtil
	LocalSpaceMap2D
	VisibilityMap
	math/BoundingBox.cc
	math/MathCommon.cc
	math/Matrix4.cc
	math/Quaternion.cc
	math/Face.cc
	math/Matrix3.cc
	math/Plane.cc
	math/Rectangle.cc

	Entity.cc
	SuperEntity.cc
	MovableEntity.cc
	Block.cc

	3DSpaceMap/SpaceMapUtil.cc
	3DSpaceMap/Block3DMapUtil.cc
	#3DSpaceMap/Block3D.cc
	#3DSpaceMap/Octree.cc
	3DSpaceMap/EntityRecorder.cc
	#3DSpaceMap/Entity3D.cc
	#3DSpaceMap/BlockEntity.cc
	3DSpaceMap/StructGraph.cc
	3DSpaceMap/Pathfinder3D.cc
	3DSpaceMap/OctomapOcTree.cc

#	MapExplorerServer.cc
)

ADD_DEPENDENCIES(SpaceMap
	opencog_atom_types
	spacetime_atom_types
	spacetime-types
)

TARGET_LINK_LIBRARIES(SpaceMap
	server
	spacetime-types
	${ATOMSPACE_query_LIBRARY}
	${ATOMSPACE_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

IF (HAVE_ZMQ)
	TARGET_LINK_LIBRARIES(SpaceMap zmq)
ENDIF (HAVE_ZMQ)

IF (HAVE_OCTOMAP)
	TARGET_LINK_LIBRARIES(SpaceMap
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY})
ENDIF (HAVE_OCTOMAP)

ADD_LIBRARY(SpaceMapUtil SHARED
	3DSpaceMap/SpaceMapUtil.cc
)

TARGET_LINK_LIBRARIES(SpaceMapUtil
	SpaceMap
	${ATOMSPACE_query_LIBRARY}
	${ATOMSPACE_LIBRARY}
)

ADD_LIBRARY(PathFinder3D SHARED
	3DSpaceMap/Pathfinder3D.cc
)

TARGET_LINK_LIBRARIES(PathFinder3D
	SpaceMap
	SpaceMapUtil
)

ADD_LIBRARY(HPASearch
	HPASearch
	QuadTree
)

ADD_LIBRARY(TangentBug
	TangentBugCommons
	TangentBug
)

ADD_EXECUTABLE (tangentbug_test TangentBugTestExec.cc)
TARGET_LINK_LIBRARIES(tangentbug_test
	TangentBug
	SpaceMap
	SpaceMapUtil
	${COGUTIL_LIBRARY}
)

# Uncomment the following line to enable ncurses printing:
#SET(TB_PRINT_NCURSES 1)

IF(TB_PRINT_NCURSES)
	ADD_DEFINITIONS(-DTB_PRINT_NCURSES)
	TARGET_LINK_LIBRARIES(tangentbug ncurses)
ENDIF(TB_PRINT_NCURSES)


###################
# AStar Directives
###################

ADD_LIBRARY (AStar
	AStarController
	LSMap2DSearchNode
	AStar3DController
	LSMap3DSearchNode
)

ADD_EXECUTABLE (astartest AStarTest)
TARGET_LINK_LIBRARIES(astartest
	TangentBug
	AStar
	SpaceMap
	SpaceMapUtil
)

 # Why is this commented out??? Do these not work any more???
 #IF(HAVE_SPATIAL_TOOLS)
 #	ADD_EXECUTABLE(MapTools
 #	MapTools
 #	MapExplorerRenderer
 #	MapExplorerSystem
 #	)
 #	TARGET_LINK_LIBRARIES(MapTools
 #	SpaceMap
 #	${COGUTIL_LIBRARY}
 #	${SDL_LIBRARY}
 #	${SDLGFX_LIBRARY}
 #	${OPENGL_LIBRARY}
 #	${Boost_PROGRAM_OPTIONS_LIBRARY}
 #	)
 #ENDIF(HAVE_SPATIAL_TOOLS)
 #
INSTALL (FILES
	Agent.h
	AStarController.h
	AStar3DController.h
	Entity.h
	fsa.h
	HPASearch.h
	HumanoidAgent.h
	LocalSpaceMap2D.h
	LocalSpaceMap2DUtil.h
	LSMap2DSearchNode.h
	LSMap3DSearchNode.h
 #	MapExplorer.h
 #	MapExplorerServer.h
	MovableEntity.h
	PetAgent.h
	Prerequisites.h
	QuadTree.h
	Block.h
	StaticEntity.h
	stlastar.h
	SuperEntity.h
	VisibilityMap.h
	DESTINATION "include/${PROJECT_NAME}/spatial"
)
INSTALL (FILES
	math/BoundingBox.h
	math/Dimension2.h
	math/Dimension3.h
	math/Face.h
	math/Line.h
	math/LineSegment.h
	math/Matrix3.h
	math/Matrix4.h
	math/Plane.h
	math/Quaternion.h
	math/Rectangle.h
	math/SquareFace.h
	math/Triangle.h
	math/Vector2.h
	math/Vector3.h
	math/Vector4.h
	DESTINATION "include/${PROJECT_NAME}/spatial/math"
)

INSTALL (FILES
	3DSpaceMap/SpaceMapUtil.h
	3DSpaceMap/Block3DMapUtil.h
#	3DSpaceMap/Block3D.h
#	3DSpaceMap/Octree.h
	3DSpaceMap/EntityRecorder.h
#	3DSpaceMap/Entity3D.h
#	3DSpaceMap/BlockEntity.h
	3DSpaceMap/Pathfinder3D.h
	3DSpaceMap/OctomapOcTree.h
	DESTINATION "include/${PROJECT_NAME}/spatial/3DSpaceMap"
)
