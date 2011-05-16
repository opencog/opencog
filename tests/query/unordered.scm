
;;
;; unordered.scm
;;
;; A very very simple test for unordered-link matching.
;; This test just has one single unordered link (the similarity link)
;; and looks for a match for it.  This being the simplest test, should
;; be able to pass.
;; 
(define (stv mean conf) (cog-new-stv mean conf))

(SimilarityLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink)
	)
)

(define (blink)
	(BindLink
		;; variable decls
		(ListLink
			(VariableNode "$var_number")
			(VariableNode "$var_schema")
		)
		(ImplicationLink
			;; body
			(SimilarityLink (stv 1.0 1.0)
				(VariableNode "$var_schema")
				(VariableNode "$var_number")
				(ExecutionOutputLink (stv 1.0 1.0)
					(VariableNode "$var_schema")
					(ListLink)
				)
			)
			;; implicand -- result
			(ListLink
				(VariableNode "$var_number")
				(VariableNode "$var_schema")
			)
		)
	)
)

