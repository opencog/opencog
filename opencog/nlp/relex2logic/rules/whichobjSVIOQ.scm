; This rule is for which objects in indirect object sentences,
; such as "Which book did you give to your friend?"
; (AN June 2015)

(define whichobjSVIOQ
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
			(dependency "_det" "$obj" "$qVar")
			(InheritanceLink
				(VariableNode "$qVar")
				(DefinedLinguisticConceptNode "which")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-whichobjSVIOQ-rule")
			(ListLink
			   (VariableNode "$subj")
			   (VariableNode "$verb")
			   (VariableNode "$obj")
			   (VariableNode "$iobj")
			)
		)
	)
)
;;ToDo: XXX FIXME define whichobjSVIOQ
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichobjSVIOQ-rule subj verb obj iobj)
	(whichobjSVIOQ-rule (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
			  (cog-name (word-inst-get-lemma verb)) (cog-name verb)
			  (cog-name (word-inst-get-lemma subj)) (cog-name subj)
			  (cog-name (word-inst-get-lemma iobj)) (cog-name iobj)
	)
)
