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
		(ImplicationLink
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
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-howdeg-q-rule")
			(ListLink
				(VariableNode "$pred")
			)
		)
	)
))

(InheritanceLink (stv 1 .99) (ConceptNode "howdeg-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "howdeg-q-Rule") howdeg-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-howdeg-q-rule pred)
	(howdegQ-rule (word-inst-get-word-str pred) (cog-name pred)
	)
)

