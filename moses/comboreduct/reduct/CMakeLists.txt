#add_library(reduct
#	action_reduction	a
#	action_rules
#	clean_reduction
#	contin_reduction
#	contin_rules
#	flat_normal_form
#	full_reduction
#	general_rules
#	logical_reduction
#	logical_rules
#	meta_rules
#	mixed_reduction
#	mixed_rules
#	perception_reduction
#	perception_rules
#	reduct
#	using
#)

#include_directories(
#${CMAKE_SOURCE_DIR}/src/
#)

#link_directories(
#${CMAKE_BINARY_DIR}/src/combo
#${CMAKE_BINARY_DIR}/src/crutil
#)

#target_link_libraries(reduct
#	crutil
#   ${LADSUTIL_LIBRARY}
#	combo
#)

#install header files
INSTALL(FILES

	action_rules.h
	branch_rules.h
	contin_rules.h
	flat_normal_form.h
	general_rules.h
	logical_rules.h
	meta_rules.h
	mixed_rules.h
	perception_rules.h
        fold_rules.h
	reduct.h

	DESTINATION

	"include/${PROJECT_NAME}/reduct" 
)

