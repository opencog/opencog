#
# Create/install MOSES man page

set(MANPAGES moses.1)

# We need gzip to make this work.
find_program(GZIP_EXECUTABLE NAMES gzip)

# Compress all manpages...
# XXX for some reason, this never notices that sources have changed...
foreach(manpage ${MANPAGES})
	add_custom_command(COMMAND ${GZIP_EXECUTABLE} -c
		${CMAKE_CURRENT_SOURCE_DIR}/${manpage} > 
      ${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz
		OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz
      )
endforeach()

# Define target so that 'make man' will do something...
add_custom_target(man ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz)

# Install to final location ...
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz
               DESTINATION ${MAN_INSTALL_DIR}/man1/)



