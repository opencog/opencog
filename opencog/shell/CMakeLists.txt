
ADD_LIBRARY (scheme-shell SHARED
   GenericShell
   SchemeShell
)

TARGET_LINK_LIBRARIES(scheme-shell
   util
   atomspace
   smob
)

INSTALL (TARGETS scheme-shell
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
   GenericShell.h
   SchemeShell.h
	DESTINATION "include/opencog/shell"
)

