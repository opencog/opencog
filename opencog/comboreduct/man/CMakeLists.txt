#
# Create/install eval-table man page

set(MANPAGES eval-table.1)

# We need gzip to make this work.
find_program(GZIP_EXECUTABLE NAMES gzip)

# Compress all manpages...
set(outlist)
foreach(manpage ${MANPAGES})
	add_custom_command(
		COMMAND ${GZIP_EXECUTABLE} -c
			${CMAKE_CURRENT_SOURCE_DIR}/${manpage} > 
			${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz
		OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz
		DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${manpage}
	)
	list(APPEND outlist ${CMAKE_CURRENT_BINARY_DIR}/${manpage}.gz)
endforeach()

# Define target so that 'make combo-man' will do something...
add_custom_target(combo-man ALL DEPENDS ${outlist})

# Install to final location ...
install(
	FILES ${outlist}
	DESTINATION ${MAN_INSTALL_DIR}/man1/
)

