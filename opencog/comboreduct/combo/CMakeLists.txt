#install header files
INSTALL(FILES

    argument.h
    action.h
    action_eval.h
    action_symbol.h
	builtin_action.h
	message.h
	action_symbol.h
	indefinite_object.h
	definite_object.h
	procedure_call.h
	procedure_repository.h
	assumption.h
	eval.h
    table.h
	perception.h
	perception_eval.h
	tree_generation.h
	type_tree_def.h
	type_tree.h
	vertex.h
	using.h
	variable_unifier.h	
	operator_base.h
	common_def.h
    simple_nn.h
    convert_ann_combo.h
    iostream_combo.h
    combo.h

	DESTINATION

	"include/${PROJECT_NAME}/combo" 
)
