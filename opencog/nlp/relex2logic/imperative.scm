(define IMPERATIVE
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
			(TypedvariableNode
				(VariableNode "$interpretation-index")
				(TypeNode "InterpretationNode")
			)
		)
		(ImplicationLink
			(AndLink
 				(WordInstanceLink
					(VariableNode "$verb")
					(VariableNode "$a-parse")
				)
				(InheritanceLink
   					(VariableNode "$verb")
   					(DefinedLinguisticConceptNode "imperative")
				)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-imp-rule")
			(ListLink
				(InterpretationNode "$interpretation-index")
			)
		)
	)
)

(InheritanceLink (stv 1 .99) (ConceptNode "IMPERATIVE-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "IMPERATIVE-Rule") IMPERATIVE)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-imp-rule int-index)
	(IMPERATIVE-rule int-index)
	)
)

