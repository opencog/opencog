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

(define implication-and-rule
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
            (GroundedSchemaNode "scm: implication-and-formula")
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

(define (implication-and-formula ACD AB BCD)
    (cog-set-tv!
        (implication-and-side-effect-free-formula ACD AB BCD)
    )
)

(define (implication-and-side-effect-free-formula ACD AB BCD)
    (stv 1 1))

; Name the rule
(define implication-and-rule-name (DefinedSchemaNode "implication-and-rule"))
(DefineLink implication-and-rule-name implication-and-rule)
