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

#ADD_EXECUTABLE (moses_service_demo moses_service_demo)
#TARGET_LINK_LIBRARIES (moses_service_demo
#        moses_service
#        moses_exec
#        moses
#        cogutil
#)

TARGET_LINK_LIBRARIES (moses_service
        moses_exec
        moses
        cogutil
)
