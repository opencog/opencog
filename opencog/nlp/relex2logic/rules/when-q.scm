; This is for when questions with verbs other than "be"
; such as "When did ytou arrive?"
; (AN June 2015)


(define when-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(dependency "_%atTime" "$verb" "$qVar")
			(AbsentLink
				(Lemma (Variable "$verb") (Word "be"))
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-when-q-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-when-q-rule verb)
	(when-rule (cog-name (word-inst-get-lemma verb)) (cog-name verb))
)
