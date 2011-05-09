
(define (stv mean conf) (cog-new-stv mean conf))

(UnorderedLink (stv 1.0 1.0)
;; (ListLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink)
	)
)

;; Note that the UnorderedLink is unordered ... 
(define (blink)
	(BindLink
		;; variable decls
		(ListLink
			(TypedVariableLink
				(VariableNode "$var_number_node_type")
				(VariableTypeNode "NumberNode")
			)
		)
		(ImplicationLink
			;; body
			(UnorderedLink
			;; (ListLink
				(VariableNode "$var_number_node_type")
				(ExecutionOutputLink
					(GroundedSchemaNode "ActivationModulatorUpdater")
					(ListLink)
				)
			)
			;; implicand -- result
			(ListLink
				(VariableNode "$var_number_node_type")
			)
		)
	)
)

