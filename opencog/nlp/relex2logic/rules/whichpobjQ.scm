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
            (WordInstanceLink
                (VariableNode "$subj")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$prep")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$pobj")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$be")
                    (VariableNode "$subj")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_obj")
                (ListLink
                    (VariableNode "$prep")
                    (VariableNode "$pobj")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_advmod")
                (ListLink
                    (VariableNode "$be")
                    (VariableNode "$prep")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_det")
                (ListLink
                    (VariableNode "$pobj")
                    (VariableNode "$qVar")
                )
            )
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
