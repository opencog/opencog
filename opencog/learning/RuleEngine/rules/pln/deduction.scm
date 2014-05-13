; =============================================================================
; Deduction Rule
;
; AND(Inheritance A B, Inheritance B C) entails Inheritance A C
; -----------------------------------------------------------------------------

(define pln-rule-deduction
    (BindLink
        (ListLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (ImplicationLink
            (AndLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C")))
            (ListLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (ExecutionLink
                    (GroundedSchemaNode "scm: pln-formula-simple-deduction")
                    (ListLink
                        (InheritanceLink
                            (VariableNode "$A")
                            (VariableNode "$B"))
                        (InheritanceLink
                            (VariableNode "$B")
                            (VariableNode "$C"))
                        (InheritanceLink
                            (VariableNode "$A")
                            (VariableNode "$C"))))))))

; -----------------------------------------------------------------------------
; Deduction Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of AC may be updated
; -----------------------------------------------------------------------------
(define (pln-formula-simple-deduction AB BC AC)
    (cog-set-tv!
        AC
        (pln-formula-simple-deduction-side-effect-free
            AB
            BC
            AC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-simple-deduction-side-effect-free AB BC AC)
    (let
        ((sB (cog-stv-strength (gdr AB)))
         (sC (cog-stv-strength (gdr BC))))
            (stv
                (if (eq?
                        0
                        (-
                            1
                            sB))
                    0 ; Set Strength to 0 if sNotB equals 0 to avoid
                      ; division by zero
                    (+                         ; Strength
                        (* (cog-stv-strength AB) (cog-stv-strength BC))
                        (/
                            (*
                                (-
                                    1
                                    (cog-stv-strength AB))
                                (-
                                    sC
                                    (*
                                        sB
                                        (cog-stv-strength BC))))
                            (-
                                1
                                sB))))
                (min
                    (cog-stv-confidence AB)
                    (cog-stv-confidence BC))))) ; Confidence

; =============================================================================
