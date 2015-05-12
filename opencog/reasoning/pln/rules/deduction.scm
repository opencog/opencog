; =============================================================================
; Deduction Rule
; TODO The rule must be applicable to ImplicationLink, SubsetLink and PartOfLink
;
; AND(Inheritance A B, Inheritance B C) entails Inheritance A C
; -----------------------------------------------------------------------------
(include "formulas.scm")
(define pln-rule-deduction
    (BindLink
        (VariableList
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C"))
        (ImplicationLink
            (AndLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B")
                )
                (InheritanceLink
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
                    (InheritanceLink
                        (VariableNode "$A")
                        (VariableNode "$B"))
                    (InheritanceLink
                        (VariableNode "$B")
                        (VariableNode "$C")
                    )
                    (InheritanceLink
                        (VariableNode "$A")
                        (VariableNode "$C")
                    )
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
        (pln-formula-simple-deduction-side-effect-free AB BC AC)
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
(define (pln-formula-simple-deduction-side-effect-free AB BC AC)
    (let
        ((sA (cog-stv-strength (gar AB)))
         (sB (cog-stv-strength (gdr AB)))
         (sC (cog-stv-strength (gdr BC)))
         (sAB (cog-stv-strength AB))
         (sBC (cog-stv-strength BC))
        )

        ; Returns sAC. Includes the consistency conditions
        (define (strength)
            (simple-deduction-formula sA sB sC sAB sBC))

        ; This is not consistant with the defintion on the wiki
        (define (confidence)
            (min
                (cog-stv-confidence AB)
                (cog-stv-confidence BC)
            )
        )

        (stv (strength) (confidence))
    )
)

; =============================================================================
