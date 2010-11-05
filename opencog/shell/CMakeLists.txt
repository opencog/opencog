
ADD_LIBRARY (scheme-shell SHARED
    GenericShell
    SchemeShell
    SchemeShellModule
)

TARGET_LINK_LIBRARIES(scheme-shell
    util
    atomspace
    smob
    ${Boost_SYSTEM_LIBRARY}
)

INSTALL (TARGETS scheme-shell
    LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
    GenericShell.h
    SchemeShell.h
    SchemeShellModule.h
    DESTINATION "include/${PROJECT_NAME}/shell"
)

