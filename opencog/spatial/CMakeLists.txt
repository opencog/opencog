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

    MapExplorerServer.cc
)

ADD_DEPENDENCIES(SpaceMap
    opencog_atom_types
)

TARGET_LINK_LIBRARIES(SpaceMap
    ${Boost_THREAD_LIBRARY}
)

ADD_LIBRARY(HPASearch
    HPASearch
    QuadTree
)

ADD_LIBRARY(TangentBug
    TangentBugCommons
    TangentBug
)

IF(COMMAND cmake_policy)
      CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_EXECUTABLE (tangentbug_test TangentBugTestExec.cc)
TARGET_LINK_LIBRARIES(tangentbug_test
    SpaceMap
    TangentBug
    util
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
    SpaceMap
    TangentBug
    AStar
    util
)

IF(Boost_PROGRAM_OPTIONS_FOUND AND OPENGL_FOUND AND SDL_FOUND AND SDLGFX_FOUND)
ADD_EXECUTABLE(MapTools
    MapTools
    MapExplorerRenderer
    MapExplorerSystem
)
TARGET_LINK_LIBRARIES(MapTools
    SpaceMap
    util
    ${SDL_LIBRARY}
    ${SDLGFX_LIBRARY}
    ${OPENGL_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY} 
)
ENDIF(Boost_PROGRAM_OPTIONS_FOUND AND OPENGL_FOUND AND SDL_FOUND AND SDLGFX_FOUND)

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
    MapExplorer.h
    MapExplorerServer.h
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

