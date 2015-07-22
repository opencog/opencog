; This rule is for subject-verb sentences, such as
; "The Lord giveth and the lord taketh away." or "You suck."
; (AN June 2015)

(define SV
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
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$subj")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$verb")
				(VariableNode "$a-parse")
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_subj")
				(ListLink
					(VariableNode "$verb")
					(VariableNode "$subj")
				)
			)
		)
   (ListLink
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-sv-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
			)
		)
   )
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-sv-rule subj verb)
 (ListLink
	(SV-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str verb) (cog-name verb)
	)
 )
)
