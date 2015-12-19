; This is for why-questions with the verb to-be, such as
; "Why are you such a scmhuck?"
; (AN June 2015)


(define why-cop-q
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
			(dependency "_%because" "$verb" "$qVar")
			(ChoiceLink
				(AndLink
			(dependency "_subj" "$verb" "$subj")
					(LemmaLink
						(VariableNode "$verb")
						(WordNode "be")
					)
				)
			(dependency "_predadj" "$verb" "$subj")
			(dependency "_psubj" "$subj" "$verb")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-why-cop-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-why-cop-q-rule subj)
	(whycop-Q-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj))
)
