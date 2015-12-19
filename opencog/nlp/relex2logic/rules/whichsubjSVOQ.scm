; This rule is for which-subject sof SVO sentences, as in
; "Which one of you ate all the ice cream?"
; (AN june 2015)

(define whichsubjSVOQ
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
	(InheritanceLink
		(VariableNode "$qVar")
		(DefinedLinguisticConceptNode "which")
	)
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-whichsubjSVOQ-rule")
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
(define (pre-whichsubjSVOQ-rule subj verb obj)
    (whichsubjSVOQ-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
              (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
              (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
    )
)
