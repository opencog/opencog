; This is for why-questions with a verb other than to-be, as in
; "Why did you sleep late?"
; (AN June 2015)

(define why-q
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
                			(DefinedLinguisticRelationshipNode "_%because")
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
			(GroundedSchemaNode "scm: pre-why-q-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
)

(InheritanceLink (stv 1 .99) (ConceptNode "why-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "why-q-Rule") why-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-why-q-rule verb)
	(why-rule (word-inst-get-word-str verb) (cog-name verb)
	)
)

