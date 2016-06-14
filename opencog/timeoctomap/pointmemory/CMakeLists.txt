        ADD_LIBRARY(point_memory SHARED
                PointMemorySCM.cc
        )
        TARGET_LINK_LIBRARIES(point_memory
                time_octomap
                ${ATOMSPACE_LIBRARIES}
        )
        INSTALL (TARGETS point_memory
                 DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
        )
        INSTALL (FILES
                 PointMemorySCM.h
                 DESTINATION "include/${PROJECT_NAME}/timeoctomap"
        )
        INSTALL (FILES
                 point-memory.scm
                 DESTINATION "share/${PROJECT_NAME}/scm/opencog/timeoctomap"
        )

        #ADD_GUILE_MODULE(point-memory.scm)

