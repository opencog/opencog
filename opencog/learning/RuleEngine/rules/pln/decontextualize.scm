; =====================================================================
; DecontextualizeRule
; 
; a)
; ContextLink <TV>
;     C
;     R A B
; |-
; R <TV>
;     C ANDLink A
;     C ANDLink B
;----------------------------------------------------------------------

(define pln-rule-decontextualize-to-and
    (BindLink
        (ListLink
            (VariableNode "$R")
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (ImplicationLink
            (ContextLink
                    (VariableNode "$C")
                    (VariableNode "$R"
                        (VariableNode "$A")
                        (VariableNode "$B")))
            (ListLink
                (VariableNode "$R"
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$A"))
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$B")))
                (ExecutionLink
                    (GroundedSchemaNode "scm: pln-formula-context")
                    (ListLink
                        (VariableNode "$R")
                        (ContextLink
                            (VariableNode "$C")
                            (VariableNode "$R"
                                (VariableNode "$A")
                                (VariableNode "$B")))))))))

; In an EvaluationLink, the PredicateNode is not 'andified'
(define pln-rule-decontextualize-to-and-evaluation
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$R")
                (VariableTypeNode "EvaluationLink"))
            (TypedVariableLink
                (VariableNode "$A")
                (VariableTypeNode "PredicateLink"))
            (VariableNode "$B")
            (VariableNode "$C"))
        (ImplicationLink
            (ContextLink
                    (VariableNode "$C")
                    (VariableNode "$R"
                        (VariableNode "$A")
                        (VariableNode "$B")))
            (ListLink
                (VariableNode "$R"
                    (VariableNode "$A"))
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$B")))
                (ExecutionLink
                    (GroundedSchemaNode "scm: pln-formula-context")
                    (ListLink
                        (VariableNode "$R")
                        (ContextLink
                            (VariableNode "$C")
                            (VariableNode "$R"
                                (VariableNode "$A")
                                (VariableNode "$B"))))))))

;----------------------------------------------------------------------
; b)
; ContextLink <TV>
;     C
;     A
; |-
; SubsetLink <TV>
;     C
;     A
;----------------------------------------------------------------------
(define pln-rule-decontextualize-to-subset
    (BindLink
        (ListLink
            (VariableNode "$A")
            (VariableNode "$C"))
        (ImplicationLink
            (ContextLink
                (VariableNode "$C")
                (VariableNode "$A"))
            (ListLink
                (SubsetLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (ExecutionLink
                    (GroundedSchemaNode "scm: pln-formula-context")
                    (ListLink
                        (SubsetLink
                            (VariableNode "$C")
                            (VariableNode "$A"))
                        (ContextLink
                            (VariableNode "$C")
                            (VariableNode "$A"))))))))

(define (pln-formula-context Relation Context)
    (cog-set-tv! Relation (cog-tv Context)))
