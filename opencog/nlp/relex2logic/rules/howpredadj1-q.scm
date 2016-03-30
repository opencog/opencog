; This rule is for how-=predicate-adjective questions, such as "How are you?" or "How was the movie?"
; (AN June 2015)


(define howpredadj1-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(dependency "_%how" "$verb" "$qVar")
			(dependency "_subj" "$verb" "$subj")
			(Lemma (Variable "$verb") (Word "be"))
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
(define-public (pre-howpredadj1-q-rule subj)
	(howpredadj-Q-rule (cog-name (word-inst-get-lemma subj)) (cog-name subj))
)
