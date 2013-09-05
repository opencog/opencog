#install header files
INSTALL(FILES
        moses_service.h
        moses_exec.h

        DESTINATION

        "include/${PROJECT_NAME}/learning/moses/service"
)

ADD_LIBRARY (moses_service
    moses_service
)

TARGET_LINK_LIBRARIES (moses_service
        moses_exec
        moses
        cogutil
        comboreduct
        feature_selection
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${Boost_THREAD_LIBRARY}
)

IF (HAVE_MPI)
    TARGET_LINK_LIBRARIES (moses_service
        # These two work for both MPICH2 and OpenMPI, for me.
        ${MPI_LIBRARY}
        ${MPI_EXTRA_LIBRARY}
    )
ENDIF (HAVE_MPI)
