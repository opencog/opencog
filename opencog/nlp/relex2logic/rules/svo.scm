; This the rule for subject-verb-object sentences, such as
; "Johnny ate the dog."
; (AN June 2015)

(define svo
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$W")
                (TypeNode "WordInstanceNode")
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
               (VariableNode "$verb-lemma")
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
               (VariableNode "$X")
               (VariableNode "$subj-lemma")
            )
            (LemmaLink
               (VariableNode "$Y")
               (VariableNode "$verb-lemma")
            )
            (LemmaLink
               (VariableNode "$Z")
               (VariableNode "$obj-lemma")
            )
            (AbsentLink
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_iobj")
                    (ListLink
                        (VariableNode "$Y")
                        (VariableNode "$W")
                    )
                )
            )
        )
; XXX FIXME ... is this ListLink wrapper really needed ???
        (ListLink
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: SVO-rule")
                (ListLink
                    (VariableNode "$subj-lemma")
                    (VariableNode "$X")
                    (VariableNode "$verb-lemma")
                    (VariableNode "$Y")
                    (VariableNode "$obj-lemma")
                    (VariableNode "$Z")
                )
            )
        )
    )
)

;;(InheritanceLink (stv 1 .99) (ConceptNode "SVO-Rule") (ConceptNode "Rule"))

;;(ReferenceLink (stv 1 .99) (ConceptNode "SVO-Rule") svo)
