#ADD_LIBRARY(crutil
#    exception
#)

#INCLUDE_DIRECTORIES(
#    ${CMAKE_SOURCE_DIR}/src
#)

#LINK_DIRECTORIES(
#    ${CMAKE_BINARY_DIR}/src/combo
#)

#TARGET_LINK_LIBRARIES(crutil
#	combo
#	${LADSUTIL_LIBRARY}
#)

#install header files
INSTALL(FILES

	exception.h

	DESTINATION

	"include/${PROJECT_NAME}/crutil"
)

