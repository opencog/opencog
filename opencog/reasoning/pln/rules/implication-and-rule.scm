; =============================================================================
; ImplicationAndRule
;
; AndLink
;   ImplicationLink
;       A
;       B
;   ImplicationLink
;       AndLink
;           B
;           C
;       D
; |-
; ImplicationLink
;   AndLink
;       A
;       C
;   D
;
; -----------------------------------------------------------------------------

(define pln-rule-implication-and
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (VariableNode "$D"))
        (AndLink
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (ImplicationLink
                (AndLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (VariableNode "$D")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-implication-and")
            (ListLink
                (ImplicationLink
                    (AndLink
                        (VariableNode "$A")
                        (VariableNode "$C"))
                    (VariableNode "$D"))
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (AndLink
                        (VariableNode "$B")
                        (VariableNode "$C"))
                    (VariableNode "$D"))))))

(define (pln-formula-implication-and ACD AB BCD)
    (cog-set-tv!
        (pln-formula-implication-and-side-effect-free ACD AB BCD)
    )
)

(define (pln-formula-implication-and-side-effect-free ACD AB BCD)
    (stv 1 1))

; Name the rule
(define pln-rule-implication-and-name (Node "pln-rule-implication-and"))
(DefineLink pln-rule-implication-and-name pln-rule-implication-and)
