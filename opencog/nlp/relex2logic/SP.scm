; This rule is for the main predicate in predicate-adjective sentences
; such as "You are very young, sir."
; (AN June 2015)


(define SP
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
				(VariableNode "$predadj")
				(TypeNode "WordInstanceNode")
			)
		)
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
   (ListLink
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-sp-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$predadj")
			)
		)
   )
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-sp-rule subj predadj)
 (ListLink
	(SV-rule (cog-name (word-inst-get-lemma subj)) (cog-name subj)
		(cog-name (word-inst-get-lemma  predadj)) (cog-name predadj)
	)
 )
)
