#install header files
INSTALL(FILES
        moses_service.h
#        moses_exec.h

        DESTINATION

        "include/${PROJECT_NAME}/learning/moses/service"
)

ADD_LIBRARY (moses_service
    moses_service
)
