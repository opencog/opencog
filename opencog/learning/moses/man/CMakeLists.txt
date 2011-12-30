#
# Create/install MOSES man page

set(MANPAGES moses.1)

# We need gzip to make this work.
find_program(GZIP_EXECUTABLE NAMES gzip)

# Compress all manpages...
# XXX for some reason, this never notices that sources have changed...
# I just don't see why this has to be so complicated, this should be easy!
# XXX cmake sucks.  autoconf rules.
#foreach(manpage ${MANPAGES})
#	add_custom_command(COMMAND ${GZIP_EXECUTABLE} -c
#		${CMAKE_CURRENT_SOURCE_DIR}/${manpage} > 
#		${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz
#	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz
#	)
#endforeach()

# OK, so removing the foreach, to get below, still doesn't work...
add_custom_command(
	COMMAND ${GZIP_EXECUTABLE} -c
		${CMAKE_CURRENT_SOURCE_DIR}/moses.1 > 
		${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz
	)

# XXX this doesn't work either.
# add_dependencies(${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz ${CMAKE_CURRENT_SOURCE_DIR}/moses.1)

# Define target so that 'make man' will do something...
# XXX except that this doesn't actually work right ... see above .. 
add_custom_target(man ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz)

# Install to final location ...
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz
               DESTINATION ${MAN_INSTALL_DIR}/man1/)



