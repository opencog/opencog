;===============================================================================
; ContextFreeToSensitiveRule
; http://wiki.opencog.org/w/ContextFreeToSensitiveRule
;
; C <TV1>
; A <TV2>
; |-
; ContextLink <TV3>
;     C
;     A
;-------------------------------------------------------------------------------

(define pln-rule-context-free-to-sensitive
    (BindLink
        (ListLink
            (VariableNode "$A")
            (VariableNode "$C"))
        (ImplicationLink
            (AndLink
                (VariableNode "$C")
                (VariableNode "$A"))
            (ExecutionLink
                (GroundedSchemaNode "scm: pln-formula-context-free-to-sensitive")
                (ListLink
                    (ContextLink
                        (VariableNode "$C")
                        (VariableNode "$A"))
                    (VariableNode "$C")
                    (VariableNode "$A"))))))

(define (pln-formula-context-free-to-sensitive Context C A)
    (cog-set-tv! Context
        (cog-new-stv
            ; strength (now just computed as the mean of the strengths of C & A)
            (/
                (+
                    (cog-stv-strength C) (cog-stv-strength A))
                2)
            ; confidence
            (*
                (cog-stv-confidence C)
                (cog-stv-confidence A)
                (- 1 (entropy (cog-stv-strength C)))
                (- 1 (entropy (cog-stv-strength A))))

(define (entropy p)
    (-
        (sum ; how should sigma be implemented here?
             ; should a uniform distribution be assumed?
            (*
                p
                (log p)))))
