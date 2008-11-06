ADD_LIBRARY (gshell SHARED
   GenericShell
	# GenericSocket.cc
	# GenericModule.cc
)

TARGET_LINK_LIBRARIES(gshell
)

INSTALL (TARGETS gshell
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	# GenericSocket.h
	# GenericModule.h
   GenericShell.h
   SchemeShell.h
	DESTINATION "include/opencog/shell"
)

ADD_LIBRARY (scheme-shell SHARED
   SchemeShell
)

TARGET_LINK_LIBRARIES(scheme-shell
   util
   atomspace
   gshell
   smob
)

