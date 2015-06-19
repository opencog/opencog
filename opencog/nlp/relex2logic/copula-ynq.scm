; This rule is for copula yes/no questions, such as . . . "Are you my mommy?"
; (AN June 2015)

(define copula-ynq
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$a-parse")
				(TypeNode "ParseNode")
			)
			(TypedVariableLink
				(VariableNode "$subj")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$verb")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$obj")
				(TypeNode "WordInstanceNode")
			)
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$subj")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$obj")
				(VariableNode "$a-parse")
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_subj")
				(ListLink
					(VariableNode "$verb")
					(VariableNode "$subj")
				)
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_obj")
				(ListLink
					(VariableNode "$verb")
					(VariableNode "$obj")
				)
			)
			(LemmaLink
				(VariableNode "$verb")
				(WordNode "be")
			)				
			(InheritanceLink
				(VariableNode "$verb")
				(DefinedLinguisticConceptNode "truth-query")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-copula-ynq-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$obj")
			)
		)
	)
)


(InheritanceLink (stv 1 .99) (ConceptNode "copula-ynq-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "copula-ynq-Rule") copula-ynq)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-copula-ynq-rule subj obj)
	(cop-ynQ-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str obj) (cog-name obj)
	)
)

