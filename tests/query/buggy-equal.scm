
bug 1520

(define pln-rule-deduction
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$A")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$B")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$C")
                (TypeNode "ConceptNode")
            )
        )
        (ImplicationLink
            (AndLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B")
                )
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C")
                )
                ; To avoid matching (Inheritance A B) and (Inheritance B
A)
                (AbsentLink
                    (EvaluationLink
                        (GroundedPredicateNode "scm: cog-equal?")
                        (ListLink
                            (VariableNode "$A")
                            (VariableNode "$C")
                        )
                    )
                )
            )
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pln-formula-simple-deduction")
                (ListLink
                    (InheritanceLink
                        (VariableNode "$A")
                        (VariableNode "$B"))
                    (InheritanceLink
                        (VariableNode "$B")
                        (VariableNode "$C")
                    )
                    (InheritanceLink
                        (VariableNode "$A")
                        (VariableNode "$C")
                    )
                )
            )
        )
    )
)



;
-----------------------------------------------------------------------------
; Check whether two nodes are equal.
;
; If they are equal then it will return TRUE_TV else it returns
FALSE_TV.
;
-----------------------------------------------------------------------------
(define (cog-equal? atom-1 atom-2)
    (if (equal? atom-1 atom-2)
        (stv 1 1)
        (stv 0 1)
    )
)
:1

