(define when-q
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$a-parse")
				(TypeNode "ParseNode")
			)
			(TypedVariableLink
				(VariableNode "$verb")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$qVar")
				(TypeNode "WordInstanceNode")
			)
		)
		(ImplicationLink
			(AndLink	
 				(WordInstanceLink
					(VariableNode "$verb")
					(VariableNode "$a-parse")
				)
				(WordInstanceLink
					(VariableNode "$qVar")
					(VariableNode "$a-parse")
				)
				(EvaluationLink
                    			(DefinedLinguisticRelationshipNode "_%atTime")
                    			(ListLink
                        			(VariableNode "$verb")
                        			(VariableNode "$qVar")
                    			)					
                		)
				(AbsentLink
					(LemmaLink
						(VariableNode "$verb")
						(WordNode "be")
					)
				)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-when-q-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
))


(InheritanceLink (stv 1 .99) (ConceptNode "when-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "when-q-Rule") when-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-when-q-rule verb)
	(when-rule (word-inst-get-word-str verb) (cog-name verb)
	)
)

