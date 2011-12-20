
# We need to put this in a library, so that MOSESUTest can link to it.
ADD_LIBRARY (moses_exec
   moses_exec
	moses_exec_def
)

ADD_EXECUTABLE (moses-exec moses-exec)
TARGET_LINK_LIBRARIES (moses-exec
	moses_exec
	moses
)
SET_TARGET_PROPERTIES(moses-exec PROPERTIES OUTPUT_NAME "moses")

# install moses-exec
INSTALL(TARGETS moses-exec RUNTIME DESTINATION bin)

#install header files
INSTALL(FILES
	moses_exec.h
	moses_exec_def.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/main" 
)
