; =============================================================================
; Modus Ponens Rule
;
; Given P(A implies B) and sA, calculate sB
; -----------------------------------------------------------------------------

(define pln-rule-modus-ponens
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (ImplicationLink
            (AndLink
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$A"))
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pln-formula-simple-modus-ponens")
                (ListLink
                    (VariableNode "$B")
                    (ImplicationLink
                        (VariableNode "$A")
                        (VariableNode "$B")))))))

; -----------------------------------------------------------------------------
; Modus Ponens Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of AC may be updated
; -----------------------------------------------------------------------------

(define (pln-formula-simple-modus-ponens B AB)
    (cog-set-tv!
        B
        (pln-formula-simple-modus-ponens-side-effect-free
            AB)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-simple-modus-ponens-side-effect-free AB)
    (let
        ((sA (cog-stv-strength (gar AB)))
         (cA (cog-stv-confidence (gar AB))))
            (stv                          ; Strength
                (*
                    (cog-stv-strength AB)
                    sA)
                (+                        ; Confidence
                    (cog-stv-confidence AB)
                    cA))))

; TODO: Complete this formulas by adding P(NOT(A) implies B)

; =============================================================================
