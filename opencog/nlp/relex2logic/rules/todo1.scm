; This rule is for "She wants to help John."  I hate these rules, which were here when I got here.
; There are five of them. There could be fifty.  There would need to be, to cover all cases.  Somebody
; randomly chose five sentence-types to enshrine in these rules.  See my comment in the old relex2logic
; rule-file for more of an explanation of the problem.
; (AN June 2015)

(define todo1
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
                (VariableNode "$subj2")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb1")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb2")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$obj")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$subj")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$verb1")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$verb2")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$obj")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$verb1")
                    (VariableNode "$subj")
                )
            )
            (AbsentLink
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_subj")
                    (ListLink
                        (VariableNode "$verb2")
                        (VariableNode "$subj2")
                    )
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_obj")
                (ListLink
                    (VariableNode "$verb2")
                    (VariableNode "$obj")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_to-do")
                (ListLink
                    (VariableNode "$verb1")
                    (VariableNode "$verb2")
                )
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pre-todo1-rule")
            (ListLink
                (VariableNode "$subj")
                (VariableNode "$verb1")
                (VariableNode "$verb2")
                (VariableNode "$obj")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-todo1-rule subj verb1 verb2 obj)
 (ListLink
  (to-do-rule-1 
    (cog-name (word-inst-get-lemma  verb1)) (cog-name verb1)
    (cog-name (word-inst-get-lemma  verb2)) (cog-name verb2)
    (cog-name (word-inst-get-lemma subj)) (cog-name subj)
    (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
  )
 )
)
