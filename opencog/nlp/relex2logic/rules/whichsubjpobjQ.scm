; This rule is for which-subjects of prepositional objects, as in
; "Which toy is in this box?"
; (AN June 2015)


(define whichsubjpobjQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$prep" "WordInstanceNode")
			(var-decl "$pobj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$prep" "$a-parse")
			(word-in-parse "$pobj" "$a-parse")
			(dependency "_predadj" "$subj" "$prep")
			(dependency "_pobj" "$prep" "$pobj")
			(dependency "_det" "$subj" "$qVar")
			(word-feat "$qVar" "which")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-whichsubjpobjQ-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$prep")
				(VariableNode "$pobj")
			)
		)
	)
)

;; XXX FIXME: define whichsubjpobjQ-rule
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-whichsubjpobjQ-rule subj prep pobj)
	(whichsubjpobjQ-rule (cog-name (word-inst-get-lemma  pobj)) (cog-name pobj)
			  (cog-name (word-inst-get-lemma prep)) (cog-name prep)
			  (cog-name (word-inst-get-lemma subj)) (cog-name subj)
	)
)
