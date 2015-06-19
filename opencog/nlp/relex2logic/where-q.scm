; This rule is for where questions without the verb "be"
; such as "Where do you go every night?"
; (AN June 2015)


(define where-q
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
                			(DefinedLinguisticRelationshipNode "_%atLocation")
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
			(GroundedSchemaNode "scm: pre-where-q-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
)


(InheritanceLink (stv 1 .99) (ConceptNode "where-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "where-q-Rule") where-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-where-q-rule verb)
	(where-rule (word-inst-get-word-str verb) (cog-name verb)
	)
)

