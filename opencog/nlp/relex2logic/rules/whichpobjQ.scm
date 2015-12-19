; This rule is for which-prepositional objects, as in
; "Which box are the drugs in?" or "In which way does this make sense?"
; (AN June 2015)

(define whichpobjQ
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$prep" "WordInstanceNode")
			(var-decl "$pobj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$be" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$prep" "$a-parse")
			(word-in-parse "$pobj" "$a-parse")
			(dependency "_subj" "$be" "$subj")
			(dependency "_obj" "$prep" "$pobj")
			(dependency "_advmod" "$be" "$prep")
			(dependency "_det" "$pobj" "$qVar")
            (InheritanceLink
                (VariableNode "$qVar")
                (DefinedLinguisticConceptNode "which")
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pre-whichpobjQ-rule")
            (ListLink
                (VariableNode "$subj")
                (VariableNode "$prep")
                (VariableNode "$pobj")
            )
        )
    )
)
;;ToDo: XXX FIXME define whichpobjQ
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichpobjQ-rule subj prep pobj)
    (whichpobjQ-rule (cog-name (word-inst-get-lemma  pobj)) (cog-name pobj)
              (cog-name (word-inst-get-lemma prep)) (cog-name prep)
              (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
    )
)
