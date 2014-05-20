ADD_SUBDIRECTORY(combo)
ADD_SUBDIRECTORY(interpreter)
ADD_SUBDIRECTORY(type_checker)
ADD_SUBDIRECTORY(table)
ADD_SUBDIRECTORY(reduct)
ADD_SUBDIRECTORY(ant_combo_vocabulary)
ADD_SUBDIRECTORY(main)
ADD_SUBDIRECTORY(crutil)

ADD_LIBRARY(comboreduct SHARED
	crutil/exception

	combo/action
	combo/action_symbol
	combo/argument
	combo/assumption
	combo/builtin_action
	combo/convert_ann_combo
	combo/definite_object
	combo/descriptions
	combo/enum_type
	combo/indefinite_object
	combo/iostream_combo
	combo/message
	combo/perception
	combo/procedure_call
	combo/procedure_repository
	combo/similarity
	combo/simple_nn
	combo/variable_unifier
	combo/vertex

	table/table
	table/table_io

	interpreter/eval
	interpreter/interpreter

	type_checker/type_tree_def
	type_checker/type_tree

	reduct/action_reduction
	reduct/action_rules
	reduct/branch_rules
	reduct/clean_reduction
	reduct/contin_reduction
	reduct/contin_rules
	reduct/flat_normal_form
	reduct/full_reduction
	reduct/general_rules
	reduct/logical_reduction
	reduct/ann_reduction
	reduct/logical_rules
	reduct/fold_reduction
	reduct/fold_rules
	reduct/meta_rules
	reduct/mixed_reduction
	reduct/mixed_rules
	reduct/perception_reduction
	reduct/perception_rules
	reduct/reduct
)

TARGET_LINK_LIBRARIES (comboreduct
	cogutil
)

ADD_LIBRARY(comboant SHARED
	ant_combo_vocabulary/ant_builtin_action
	ant_combo_vocabulary/ant_perception
	ant_combo_vocabulary/ant_action_symbol
	ant_combo_vocabulary/ant_indefinite_object
	ant_combo_vocabulary/ant_combo_vocabulary
)

TARGET_LINK_LIBRARIES (comboant
	cogutil
)


#install library
IF (WIN32)
	INSTALL(TARGETS comboreduct DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
	INSTALL(TARGETS comboreduct
           LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}"   # lib*.so files
           ARCHIVE DESTINATION "lib${LIB_DIR_SUFFIX}")  # lib*.a files
ENDIF (WIN32)
