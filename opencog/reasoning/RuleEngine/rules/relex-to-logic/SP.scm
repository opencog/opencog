(define SP
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
				(VariableNode "$predadj")
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
					(VariableNode "$predadj")
					(VariableNode "$a-parse")
				)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_predadj")
					(ListLink
						(VariableNode "$subj")
						(VariableNode "$predadj")
 					)
				)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-sp-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$predadj")
			)
		)
	)
))


(InheritanceLink (stv 1 .99) (ConceptNode "SP-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "SP-Rule") SP)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-sp-rule subj predadj)
	(SV-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str predadj) (cog-name predadj)
	)
)

