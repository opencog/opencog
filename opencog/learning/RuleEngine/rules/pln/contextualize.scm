; =====================================================================
; ContextualizeRule
; (http://wiki.opencog.org/w/ContextualizerRule)
; 
; a)
; R <TV>
;    C ANDLink A
;    C ANDLink B
; |-
; ContextLink <TV>
;    C
;    R A B
;----------------------------------------------------------------------

(define pln-rule-contextualize-implication
    (BindLink
        (ListLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
            (ImplicationLink
                (ImplicationLink
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$A"))
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$B")))
                (ListLink
                    (ContextLink 
                        (VariableNode "$C")
                        (InheritanceLink
                            (VariableNode "$A")
                            (VariableNode "$B")))
                    (ExecutionLink
                        (GroundedSchemaNode "scm: pln-formula-context")
                        (ListLink
                            (ContextLink
                                (VariableNode "$C")
                                (InheritanceLink
                                    (VariableNode "$A")
                                    (VariableNode "$B")))
                            (ImplicationLink
                                (AndLink
                                    (VariableNode "$C")
                                    (VariableNode "$A"))
                                (AndLink
                                    (VariableNode "$C")
                                    (VariableNode "$B")))))))))


; In an EvaluationLink, the PredicateNode is not 'andified'
(define pln-rule-contextualize-evaluation
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$A")
                (VariableTypeNode "PredicateNode"))
            (VariableNode "$B")
            (VariableNode "$C"))
            (ImplicationLink
                (EvaluationLink
                    (VariableNode "$A")
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$B"))
                (ListLink
                    (ContextLink
                        (VariableNode "$C")
                        (EvaluationLink
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
                            (EvaluationLink
                                (VariableNode "$A")
                                (AndLink
                                    (VariableNode "$C")
                                    (VariableNode "$B"))))))))))

;----------------------------------------------------------------------
; b)
; SubsetLink <TV>
;    C
;    A
; |-
; ContextLink <TV>
;    C
;    A
;----------------------------------------------------------------------
(define pln-rule-contextualize-subset
    (BindLink
        (ListLink
            (VariableNode "$C")
            (VariableNode "$A"))
        (ImplicationLink
            (SubsetLink
                (VariableNode "$C")
                (VariableNode "$A"))
            (ListLink
                (ContextLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (ExecutionLink
                    (GroundedSchemaNode "scm:pln-formula-context")
                    (ListLink
                        (ContextLink
                            (VariableNode "$C")
                            (VariableNode "$A"))
                        (SubsetLink
                            (VariableNode "$C")
                            (VariableNode "$A"))))))))


(define (pln-formula-context Context Relation)
    (cog-set-tv! Context (cog-tv Relation)))
