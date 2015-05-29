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
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-howquant-q-rule")
			(ListLink
				(VariableNode "$noun")
			)
		)
	)
)

(InheritanceLink (stv 1 .99) (ConceptNode "howquant-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "howquant-q-Rule") howquant-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-howquant-q-rule noun)
	(howquantQ-rule (word-inst-get-word-str noun) (cog-name noun)
	)
)

