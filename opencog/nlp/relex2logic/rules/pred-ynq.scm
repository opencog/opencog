; This rule is for yes/no questions with verbs other than "be"
; such as "Did you sleep well?"
; (AN June 2015)


(define pred-ynq
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
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$verb")
				(VariableNode "$a-parse")
			)
			(InheritanceLink
				(VariableNode "$verb")
				(DefinedLinguisticConceptNode "truth-query")
			)
		)
   (ListLink
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-pred-ynq-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
   )
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-pred-ynq-rule verb)
 (ListLink
	(pred-ynQ-rule (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
	)
 )
)
