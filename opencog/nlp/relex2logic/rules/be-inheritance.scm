; This rule handles sentences like "Bob is green." "Bob is my friend." etc. In other words, anything using the verb "be".
; There probably should be a logical difference between be-inheritance and copula, and perhaps other uses of "be" but the
; _copula relation doesn't get assigned by relex as far as I can tell.
; (AN June 2015)

(define be-inheritance
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$X")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$Y")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$Z")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
               (VariableNode "$subj-lemma")
               (TypeNode "WordNode")
            )
            (TypedVariableLink
               (VariableNode "$obj-lemma")
               (TypeNode "WordNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$X")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$Y")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$Z")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$Y")
                    (VariableNode "$X")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_obj")
                (ListLink
                    (VariableNode "$Y")
                    (VariableNode "$Z")
                )
            )
            (LemmaLink
                (VariableNode "$Y")
                (WordNode "be")
            )
            (LemmaLink
               (VariableNode "$X")
               (VariableNode "$subj-lemma")
            )
            (LemmaLink
               (VariableNode "$Z")
               (VariableNode "$obj-lemma")
            )
        )
; XXX FIXME ... is this ListLink wrapper really needed ???
       (ListLink
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: be-inheritance-rule")
            (ListLink
                (VariableNode "$subj-lemma")
                (VariableNode "$X")
                (VariableNode "$obj-lemma")
                (VariableNode "$Z")
            )
        )
      )
    )
)
