; ================================
; ContextualizeRule
; 
; a)
; R <TV>
;    C ANDLink A
;    C ANDLink B
; |-
; ContextLink <TV>
;    C
;    R A B
;----------------------------------

(define pln-rule-contextualize
    (BindLink
        (ListLink
            (VariableNode "$R")
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (if (cog-link? (VariableNode "$R"))     
            (ImplicationLink
                (VariableNode "$R"
                    (if (eq? 'EvaluationLink (cog-type VariableNode "$R"))
                        (if (eq? 'PredicateNode (cog-type VariableNode "$A"))
                            (VariableNode "$A"))
                            else (AndLink
                                    (VariableNode "$C")
                                    (VariableNode "$A")))
                            (AndLink
                                (VariableNode "$C")
                                (VariableNode "$B")))
                (ListLink
                    (ContextLink 
                        (VariableNode "$C")
                        (VariableNode "$R"
                            (VariableNode "$A")
                            (VariableNode "$B")))
                    (ExecutionLink
                        (GroundedSchemaNode "scm: pln-formula-context")
                        (ListLink
                            (ContextLink
                                (VariableNode "$C")
                                (VariableNode "$R"
                                    (VariableNode "$A")
                                    (VariableNode "$B")))
                            (VariableNode "$R"))))))))
                    
(define (pln-formula-context Context Relation)
    (cog-set-tv! Context (cog-tv Relation)))
