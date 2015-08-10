; =============================================================================
; EquivalenceTransformationRule
; 
; EquivalenceLink
;   A
;   B
; |-
; AndLink
;   ImplicationLink
;       A
;       B
;   ImplicationLink
;       B
;       A
;
; -----------------------------------------------------------------------------

(define pln-rule-equivalence-transformation
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (EquivalenceLink
            (VariableNode "$A")
            (VariableNode "$B"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm:pln-formula-equivalence-transformation")
            (ListLink
                (AndLink
                    (ImplicationLink
                        (ConceptNode "$A")
                        (ConceptNode "$B"))
                    (ImplicationLink
                        (ConceptNode "$B")
                        (ConceptNode "$A")))
                (EquivalenceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))))

(define (pln-formula-equivalence-transofrmation AII EV)
    (cog-set-tv!
        AII (cog-tv EV)))
