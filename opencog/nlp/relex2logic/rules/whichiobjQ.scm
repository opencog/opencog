; This is for which-indirect objects as in "Which man did you give the
; message to?" or "To which man did you give the message?"
;
; This rule would be correct but doesn't fire because the relex output
; doesn't output an _iobj or a _pobj for it, due to the syntax.  No way
; to write a set of conditions that will work for this one until I fix
; the relex error -- AN June 2015

(define whichiobjQ
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
			(dependency "_iobj" "$verb" "$iobj")
			(dependency "_det" "$iobj" "$qVar")
			(word-feat "$qVar" "which")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-whichiobjQ-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
				(VariableNode "$obj")
				(VariableNode "$iobj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-whichiobjQ-rule subj verb obj iobj)
	(whichiobjQ-rule (cog-name (word-inst-get-lemma subj)) (cog-name subj)
			  (cog-name (word-inst-get-lemma verb)) (cog-name verb)
			  (cog-name (word-inst-get-lemma obj)) (cog-name obj)
			  (cog-name (word-inst-get-lemma  iobj)) (cog-name iobj)
	)
)
