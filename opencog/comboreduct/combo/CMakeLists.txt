#install header files
INSTALL(FILES

	action.h
	action_eval.h
	action_symbol.h
	argument.h
	assumption.h
	builtin_action.h
	combo.h
	common_def.h
	convert_ann_combo.h
	definite_object.h
	enum_type.h
	eval.h
    interpreter.h
	indefinite_object.h
	iostream_combo.h
	message.h
	operator_base.h
	perception.h
	perception_eval.h
	procedure_call.h
	procedure_repository.h
	simple_nn.h
	table.h
	table_io.h
	tree_generation.h
	type_tree.h
	type_tree_def.h
	using.h
	variable_unifier.h	
	vertex.h

	DESTINATION

	"include/${PROJECT_NAME}/combo" 
)
