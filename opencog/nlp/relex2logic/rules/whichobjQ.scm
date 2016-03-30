; This rule is for which-objects as in
; "Which book did you read?"
; (AN June 2015)

(define whichobjQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(dependency "_subj" "$verb" "$subj")
			(dependency "_obj" "$verb" "$obj")
			(dependency "_det" "$obj" "$qVar")
			(word-feat "$qVar" "which")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-whichobjQ-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
				(VariableNode "$obj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-whichobjQ-rule subj verb obj)
	(whichobjQ-rule (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
			  (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
			  (cog-name (word-inst-get-lemma subj)) (cog-name subj)
	)
)
