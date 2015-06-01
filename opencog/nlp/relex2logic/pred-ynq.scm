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
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-pred-ynq-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
)


(InheritanceLink (stv 1 .99) (ConceptNode "pred-ynq-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "pred-ynq-Rule") pred-ynq)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-pred-ynq-rule verb)
	(pred-ynQ-rule (word-inst-get-word-str verb) (cog-name verb)
	)
)

