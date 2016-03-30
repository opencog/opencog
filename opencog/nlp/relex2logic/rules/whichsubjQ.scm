; This rule is for which-subjects of SVO sentences, as in
; "Which guy ate all the pizza?"
; (AN June 2015)

(define whichsubjQ
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
			(dependency "_det" "$subj" "$qVar")
			(word-feat "$qVar" "which")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-whichsubjQ-rule")
			  (ListLink
				 (VariableNode "$subj")
				 (VariableNode "$verb")
				 (VariableNode "$obj")
			)
		)
	)
)
;ToDo: XXX FIXME define whichsubjQ-rule
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-whichsubjQ-rule subj verb obj)
	(whichsubjQ-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
			  (cog-name (word-inst-get-lemma verb)) (cog-name verb)
			  (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
	)
)
