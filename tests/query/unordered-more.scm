;;
;; unordered-more.scm
;;
;; A slightly more complicated unordered-link test case; 
;; has some confounding graphs that should not be found ...

(define (stv mean conf) (cog-new-stv mean conf))

;; should match to this.
(SimilarityLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink)
	)
)

;; this should not match.
(SimilarityLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(SemeNode "We are legion; we are Anonymous")
		(ListLink)
	)
)

;; this should not match.
(SimilarityLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(ConceptNode "ring a ling a ding")
		)
	)
)

;; this should not match.
(SimilarityLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(AtTimeLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink)
	)
)

;; this should not match.
(UnorderedLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink)
	)
)

;; this should not match.
(SimilarityLink (stv 1.0 1.0)
	(WordNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink)
	)
)

;; Note that the SimilarityLink is unordered ... 
(define (blank)
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
			(SimilarityLink
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

