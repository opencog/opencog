; This the rule for subject-verb-object sentences, such as
; "Johnny ate the dog."
; (AN June 2015)

(define svo
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$W" "WordInstanceNode")
			(var-decl "$X" "WordInstanceNode")
			(var-decl "$Y" "WordInstanceNode")
			(var-decl "$Z" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb-lemma" "WordNode")
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

;;(InheritanceLink (stv 1 .99) (ConceptNode "SVO-Rule") (ConceptNode "Rule"))

;;(ReferenceLink (stv 1 .99) (ConceptNode "SVO-Rule") svo)
