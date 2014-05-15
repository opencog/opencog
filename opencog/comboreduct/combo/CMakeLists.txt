#install header files
INSTALL(FILES

	action.h
	action_symbol.h
	argument.h
	assumption.h
	builtin_action.h
	combo.h
	common_def.h
	convert_ann_combo.h
	definite_object.h
	enum_type.h
	indefinite_object.h
	iostream_combo.h
	message.h
	operator_base.h
	perception.h
	procedure_call.h
	procedure_repository.h
	simple_nn.h
	tree_generation.h
	variable_unifier.h	
	vertex.h

	DESTINATION

	"include/${PROJECT_NAME}/combo" 
)
