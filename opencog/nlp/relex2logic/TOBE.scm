(define TOBE
	(BindLink
		(ListLink
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
				(VariableNode "$adj")
				(TypeNode "WordInstanceNode")
			)
		)
		(ImplicationLink
			(AndLink
				(WordInstanceLink
					(VariableNode "$subj")
					(VariableNode "$a-parse")
				)
 				(WordInstanceLink
					(VariableNode "$verb")
					(VariableNode "$a-parse")
				)
 				(WordInstanceLink
					(VariableNode "$adj")
					(VariableNode "$a-parse")
				)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_to-be")
					(ListLink
						(VariableNode "$verb")
						(VariableNode "$adj")
 					)
				)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_subj")
					(ListLink
						(VariableNode "$verb")
						(VariableNode "$subj")
 					)
				)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-tobe-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
				(VariableNode "$adj")
			)
		)
	)
))

(InheritanceLink (stv 1 .99) (ConceptNode "TOBE-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "TOBE-Rule") SP)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-tobe-rule subj verb adj)
	(to-be-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str verb) (cog-name verb)
		(word-inst-get-word-str adj) (cog-name adj)
	)
)

