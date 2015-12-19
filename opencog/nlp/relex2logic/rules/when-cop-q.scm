; This rule is for when copula questions, such as "When is the meeting."
; Well actually that's not a copula but let's not get into that, okay. You get the idea anyway.
; (AN June 2015)

(define when-cop-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$subj" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(word-in-parse "$subj" "$a-parse")
			(dependency "_%atTime" "$verb" "$qVar")
			(dependency "_subj" "$verb" "$subj")
			(LemmaLink
				(VariableNode "$verb")
				(WordNode "be")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-when-cop-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-when-cop-q-rule subj)
	(whencop-Q-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj))
)
