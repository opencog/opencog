ADD_SUBDIRECTORY(combo)
ADD_SUBDIRECTORY(reduct)
ADD_SUBDIRECTORY(ant_combo_vocabulary)
ADD_SUBDIRECTORY(main)
ADD_SUBDIRECTORY(crutil)

ADD_LIBRARY(comboreduct SHARED
	crutil/exception

	combo/action
	combo/builtin_action
	combo/message	
	combo/action_symbol
	combo/definite_object
	combo/indefinite_object
	combo/procedure_call
	combo/procedure_repository
	combo/assumption
	combo/eval
	combo/table
	combo/perception
	combo/perception_eval
	combo/tree_generation
	combo/type_tree_def
	combo/type_tree
	combo/vertex
	combo/using
	combo/variable_unifier
	combo/simple_nn
	combo/convert_ann_combo

	reduct/action_reduction
	reduct/action_rules
	reduct/clean_reduction
	reduct/contin_reduction
	reduct/contin_rules
	reduct/flat_normal_form
	reduct/full_reduction
	reduct/general_rules
	reduct/logical_reduction
	reduct/ann_reduction
	reduct/logical_rules
	reduct/meta_rules
	reduct/mixed_reduction
	reduct/mixed_rules
	reduct/perception_reduction
	reduct/perception_rules
	reduct/reduct
	reduct/using

	ant_combo_vocabulary/ant_builtin_action
	ant_combo_vocabulary/ant_perception
	ant_combo_vocabulary/ant_action_symbol
	ant_combo_vocabulary/ant_indefinite_object
	ant_combo_vocabulary/ant_combo_vocabulary
)

TARGET_LINK_LIBRARIES (comboreduct
	util
)

#install library
IF (WIN32)
	INSTALL(TARGETS comboreduct DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
	INSTALL(TARGETS comboreduct LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)
