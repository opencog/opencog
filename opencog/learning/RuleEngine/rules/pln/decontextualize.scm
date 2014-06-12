; =====================================================================
; DecontextualizeRule
; http://wiki.opencog.org/w/DecontextualizerRule
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

; Inverse of (pln-rule-contextualize-inheritance):
; a ContextLink consisting of an InheritanceLink is decontextualized,
; i.e. reduced to an inheritance relationship
(define pln-rule-decontextualize-inheritance
    (BindLink
        (ListLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (ImplicationLink
            (ContextLink
                (VariableNode "$C")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B")))
            (ExecutionLink
                (GroundedSchemaNode "scm: pln-formula-context")
                (ListLink
                    (InheritanceLink
                        (AndLink
                            (VariableNode "$C")
                            (VariableNode "$A"))
                        (AndLink
                            (VariableNode "$C")
                            (VariableNode "$B")))
                    (ContextLink
                        (VariableNode "$C")
                        (InheritanceLink
                            (VariableNode "$A")
                            (VariableNode "$B"))))))))

; Inverse of (pln-rule-contextualize-evaluation):
; in an EvaluationLink, the PredicateNode is not 'andified';
; gets rid of the ContextLink enclosing an EvaluationLink
(define pln-rule-decontextualize-evaluation
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$A")
                (VariableTypeNode "PredicateNode"))
            (VariableNode "$B")
            (VariableNode "$C"))
        (ImplicationLink
            (ContextLink
                (VariableNode "$C")
                (EvaluationLink
                    (VariableNode "$A")
                    (ListLink
                        (VariableNode "$B"))))
            (ExecutionLink
                (GroundedSchemaNode "scm: pln-formula-context")
                (ListLink
                    (EvaluationLink
                        (VariableNode "$A")
                        (ListLink
                            (AndLink
                                (VariableNode "$C")
                                (VariableNode "$B"))))
                    (ContextLink
                        (VariableNode "$C")
                        (EvaluationLink
                            (VariableNode "$A")
                            (ListLink
                                (VariableNode "$B")))))))))

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
; Inverse of (pln-rule-contextualize-subset):
; ContextLink is transformed into a SubsetLink
(define pln-rule-decontextualize-subset
    (BindLink
        (ListLink
            (VariableNode "$A")
            (VariableNode "$C"))
        (ImplicationLink
            (ContextLink
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
                        (VariableNode "$A")))))))

(define (pln-formula-context Relation Context)
    (cog-set-tv! Relation (cog-tv Context)))
