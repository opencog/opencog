; This rule is for where questions without the verb "be"
; such as "Where do you go every night?"
; (AN June 2015)


(define where-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
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

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-where-q-rule verb)
	(where-rule (cog-name (word-inst-get-lemma  verb)) (cog-name verb))
)
