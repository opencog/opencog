        ADD_LIBRARY(point_memory
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

        ADD_GUILE_MODULE(point-memory.scm)

