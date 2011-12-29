#
# Create/install MOSES man page

set(MANPAGES moses.1)

# We need gzip to make this work.
find_program(GZIP_EXECUTABLE NAMES gzip)

# Compress all manpages...
# XXX this is somehow broken, the gzipped version is not rebuilt...
foreach(manpage ${MANPAGES})
	execute_process(COMMAND ${GZIP_EXECUTABLE} -c
		${CMAKE_CURRENT_SOURCE_DIR}/${manpage} 
		OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz
      )
endforeach()

# Define target so that 'make man' will do something...
add_custom_target(man ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz)

# Install to final location ...
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moses.1.gz
               DESTINATION ${MAN_INSTALL_DIR}/man1/)



