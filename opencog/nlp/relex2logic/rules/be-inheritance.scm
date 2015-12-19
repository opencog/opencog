; This rule handles sentences like "Bob is green." "Bob is my friend." etc. In other words, anything using the verb "be".
; There probably should be a logical difference between be-inheritance and copula, and perhaps other uses of "be" but the
; _copula relation doesn't get assigned by relex as far as I can tell.
; (AN June 2015)

(define be-inheritance
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$X" "WordInstanceNode")
			(var-decl "$Y" "WordInstanceNode")
			(var-decl "$Z" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$obj-lemma" "WordNode")
        )
        (AndLink
			(word-in-parse "$X" "$a-parse")
			(word-in-parse "$Y" "$a-parse")
			(word-in-parse "$Z" "$a-parse")
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
