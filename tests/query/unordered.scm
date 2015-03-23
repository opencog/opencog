
;;
;; unordered.scm
;;
;; A very very simple test for unordered-link matching.
;; This test just has one single unordered link (the similarity link)
;; and looks for a match for it.  This being the simplest test, and
;; should be easy to pass.
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

;; We expect the below to match in two different possible ways:
;; $var_one might bind to either GroundedSchemaNode or the 
;; NumberNode, while $var_two will bind to the other one.
(define (pair)
	(BindLink
		;; variable decls
		(ListLink
			(VariableNode "$var_one")
			(VariableNode "$var_two")
		)
		(ImplicationLink
			;; body
			(SimilarityLink (stv 1.0 1.0)
				(VariableNode "$var_one")
				(VariableNode "$var_two")
				(ExecutionOutputLink (stv 1.0 1.0)
					(GroundedSchemaNode "ActivationModulatorUpdater")
					(ListLink)
				)
			)
			;; implicand -- result
			(ListLink
				(VariableNode "$var_one")
				(VariableNode "$var_two")
			)
		)
	)
)

;; We expect the below to match in only one way: the grounding
;; of $var_schema must be to GroundedSchemaNode, due to it's
;; presence in the ExecutionOutputLink.  Thus, $var_number must,
;; by elimination, be grounded by the NumberNode.
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

;; Same as above, except that we reverse order of the two
;; variables in the body.  Despite this, the one valid grounding
;; must be found.
(define (blinker)
	(BindLink
		;; variable decls
		(ListLink
			(VariableNode "$vbr_schema")
			(VariableNode "$vbr_number")
		)
		(ImplicationLink
			;; body
			(SimilarityLink (stv 1.0 1.0)
				(VariableNode "$vbr_schema")
				(VariableNode "$vbr_number")
				(ExecutionOutputLink (stv 1.0 1.0)
					(VariableNode "$vbr_schema")
					(ListLink)
				)
			)
			;; implicand -- result
			(ListLink
				(VariableNode "$vbr_number")
				(VariableNode "$vbr_schema")
			)
		)
	)
)

