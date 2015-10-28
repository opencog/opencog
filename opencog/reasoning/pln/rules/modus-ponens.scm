; =============================================================================
; Modus Ponens Rule
;
; Given P(A implies B) and sA, calculate sB
; -----------------------------------------------------------------------------

(define modus-ponens-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: simple-modus-ponens-formula")
            (ListLink
                (VariableNode "$B")
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))))

; -----------------------------------------------------------------------------
; Modus Ponens Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of AC may be updated
; -----------------------------------------------------------------------------

(define (simple-modus-ponens-formula B AB)
    (cog-set-tv!
        B
        (simple-modus-ponens-side-effect-free-formula
            AB)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (simple-modus-ponens-side-effect-free-formula AB)
    (let
        ((sA (cog-stv-strength (gar AB)))
         (cA (cog-stv-confidence (gar AB))))
            (stv                          ; Strength
                (*
                    (cog-stv-strength AB)
                    sA)
                (min                      ; Confidence
                    (cog-stv-confidence AB)
                    cA))))

; TODO: Complete this formulas by adding P(NOT(A) implies B)

; =============================================================================

; Name the rule
(define modus-ponens-rule-name (Node "modus-ponens-rule"))
(DefineLink modus-ponens-rule-name modus-ponens-rule)
