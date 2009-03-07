#install header files
INSTALL(FILES

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
	complexity.h
	eval.h
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

	DESTINATION

	"include/${PROJECT_NAME}/combo" 
)

