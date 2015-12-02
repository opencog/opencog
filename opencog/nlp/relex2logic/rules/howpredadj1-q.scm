; This rule is for how-=predicate-adjective questions, such as "How are you?" or "How was the movie?"
; (AN June 2015)


(define howpredadj1-q
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
				(VariableNode "$qVar")
				(TypeNode "WordInstanceNode")
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
			(WordInstanceLink
				(VariableNode "$subj")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$qVar")
				(VariableNode "$a-parse")
			)
			(EvaluationLink
     			(DefinedLinguisticRelationshipNode "_%how")
     			(ListLink
        			(VariableNode "$verb")
        			(VariableNode "$qVar")
     			)
     		)
			(EvaluationLink
     			(DefinedLinguisticRelationshipNode "_subj")
     			(ListLink
        			(VariableNode "$verb")
        			(VariableNode "$subj")
     			)
     		)
			(LemmaLink
				(WordInstanceNode "$verb")
				(WordNode "be")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-howpredadj1-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-howpredadj1-q-rule subj)
	(howpredadj-Q-rule (cog-name (word-inst-get-lemma subj)) (cog-name subj))
)
