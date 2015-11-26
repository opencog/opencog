; This rule is for "how of degree questions" such as "How fast can you run?"
; (AN June 2015)

(define howdeg-q
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$a-parse")
				(TypeNode "ParseNode")
			)
			(TypedVariableLink
				(VariableNode "$qVar")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$pred")
				(TypeNode "WordInstanceNode")
			)
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$qVar")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$pred")
				(VariableNode "$a-parse")
			)
			(EvaluationLink
                			(DefinedLinguisticRelationshipNode "_%howdeg")
                			(ListLink
                    			(VariableNode "$pred")
                    			(VariableNode "$qVar")
                			)
            		)
			(InheritanceLink
				(VariableNode "$qVar")
				(DefinedLinguisticConceptNode "how_much")
			)
		)
 (ListLink
	(ExecutionOutputLink
		(GroundedSchemaNode "scm: pre-howdeg-q-rule")
		(ListLink
			(VariableNode "$pred")
		)
	)
 )
))

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-howdeg-q-rule pred)
 (ListLink
	(howdegQ-rule (cog-name (word-inst-get-lemma pred)) (cog-name pred)
	)
 )
)
