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

(define equivalence-transformation-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (EquivalenceLink
            (VariableNode "$A")
            (VariableNode "$B"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: equivalence-transformation-formula")
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

(define (equivalence-transformation-formula AII EV)
    (cog-set-tv!
        AII (cog-tv EV)))

; Name the rule
(define equivalence-transformation-rule-name
  (Node "equivalence-transformation-rule"))
(DefineLink
  equivalence-transformation-rule-name
  equivalence-transformation-rule)
