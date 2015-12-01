; This rule is for which-prepositional objects, as in
; "Which box are the drugs in?" or "In which way does this make sense?"
; (AN June 2015)

(define whichpobjQ
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$subj")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$prep")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$pobj")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$qVar")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$be")
                (TypeNode "WordInstanceNode")
            )
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
;;ToDo: define whichpobjQ
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichpobjQ-rule subj prep pobj)
  (ListLink
    (whichpobjQ-rule (cog-name (word-inst-get-lemma  pobj)) (cog-name pobj)
              (cog-name (word-inst-get-lemma prep)) (cog-name prep)
              (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
    )
  )
)
