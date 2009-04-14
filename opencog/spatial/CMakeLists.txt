IF(CMAKE_COMPILER_IS_GNUCXX)
  # Best compromise between compile time and performance:
  # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -finline-functions")

  # Fastest:
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -fno-inline")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ADD_LIBRARY(SpaceMap
    LocalSpaceMap2DUtil
    LocalSpaceMap2D
    VisibilityMap
    Math/BoundingBox.cc  
    Math/MathCommon.cc  
    Math/Matrix4.cc  
    Math/Quaternion.cc
    Math/Face.cc         
    Math/Matrix3.cc     
    Math/Plane.cc    
    Math/Rectangle.cc

    Entity.cc
    SuperEntity.cc
    MovableEntity.cc
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

ADD_EXECUTABLE (tangentbug TangentBugTestExec.cc)
TARGET_LINK_LIBRARIES(tangentbug
    Control
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
)

ADD_EXECUTABLE (astartest AStarTest)
TARGET_LINK_LIBRARIES(astartest
    Control
    SpaceMap
    TangentBug
    AStar
    util
)

# uncomment the lines below to compile MapViewer. You must have libsdl and libsdl_gfx
# installed on your system

#ADD_EXECUTABLE(MapViewer
#	MapViewer
#	Renderer
#)
#TARGET_LINK_LIBRARIES(MapViewer
#    Control
#    SpaceMap
#    ${LADSUTIL_LIBRARY}
#    HPASearch
#    SDL
#    SDL_gfx
#)

