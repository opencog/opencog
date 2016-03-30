; This rule is for which-subjects of SVIO sentences, such as
; "Which agent sent you this message?"
; (AN June 2015)

(define whichsubjSVIOQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$iobj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(word-in-parse "$iobj" "$a-parse")
			(dependency "_subj" "$verb" "$subj")
			(dependency "_obj" "$verb" "$obj")
			(dependency "_iobj" "$verb" "$iobj")
			(dependency "_det" "$subj" "$qVar")
			(word-feat "$qVar" "which")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-whichsubjSVIOQ-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
				(VariableNode "$obj")
			)
		)
	)
)
;ToDo: XXX FIXME define whichsubjSVIOQ-rule
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-whichsubjSVIOQ-rule subj verb obj iobj)
	(whichsubjSVIOQ-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
		(cog-name (word-inst-get-lemma verb)) (cog-name verb)
		(cog-name (word-inst-get-lemma obj)) (cog-name obj)
		(cog-name (word-inst-get-lemma  iobj)) (cog-name iobj)
	)
)
