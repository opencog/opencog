; =============================================================================
; Deduction Rule
;
; A->B
; B->C
; |-
; A->C
; -----------------------------------------------------------------------------

(load "formulas.scm")

(define pln-rule-deduction
    (BindLink
        (VariableList
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C"))
        (AndLink
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B")
            )
            (ImplicationLink
                (VariableNode "$B")
                (VariableNode "$C")
            )
            ; To avoid matching (Inheritance A B) and (Inheritance B A)
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$C")
                )
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-simple-deduction")
            (ListLink
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (VariableNode "$B")
                    (VariableNode "$C")
                )
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$C")
                )
            )
        )
    )
)


; -----------------------------------------------------------------------------
; Deduction Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of AC may be updated
; -----------------------------------------------------------------------------
(define (pln-formula-simple-deduction AB BC AC)
    (cog-set-tv!
        AC
        (pln-formula-simple-deduction-side-effect-free AB BC)
    )
)

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; It is a partial implementation of the forumula @
; http://wiki.opencog.org/w/Independence_Based_Deduction_Formula
;
; TODO The confidence calcualtion is not inline with the wiki as the details
;      not clearly defined there.
; -----------------------------------------------------------------------------
(define (pln-formula-simple-deduction-side-effect-free AB BC)
    (let*
        ((sA (cog-stv-strength (gar AB)))
         (sB (cog-stv-strength (gdr AB)))
         (sC (cog-stv-strength (gdr BC)))
         (sAB (cog-stv-strength AB))
         (sBC (cog-stv-strength BC))
         (cAB (cog-stv-confidence AB))
         (cBC (cog-stv-confidence BC))
         (strength (simple-deduction-formula sA sB sC sAB sBC))
         (confidence (min cAB cBC)))
      ;; (display "sA = ") (display sA)
      ;; (display ", sB = ") (display sB)
      ;; (display ", sC = ") (display sC)
      ;; (display ", sAB = ") (display sAB)
      ;; (display ", sBC = ") (display sBC)
      ;; (display ", cAB = ") (display cAB)
      ;; (display ", cBC = ") (display cBC)
      ;; (display ", strength = ") (display strength)
      ;; (display ", confidence = ") (display confidence)
      (stv strength confidence)
    )
)


; =============================================================================
