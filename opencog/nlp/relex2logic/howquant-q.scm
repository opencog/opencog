; This is for how-of-quantity questions, such as "How much did that bong cost?"
; (AN June 2015)

(define howquant-q
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
				(VariableNode "$noun")
				(TypeNode "WordInstanceNode")
			)
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$qVar")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$noun")
				(VariableNode "$a-parse")
			)
			(EvaluationLink
                			(DefinedLinguisticRelationshipNode "_quantity")
                			(ListLink
                    			(VariableNode "$noun")
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
			(GroundedSchemaNode "scm: pre-howquant-q-rule")
			(ListLink
				(VariableNode "$noun")
			)
		)
   )
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-howquant-q-rule noun)
 (ListLink
	(howquantQ-rule (word-inst-get-word-str noun) (cog-name noun)
	)
 )
)
