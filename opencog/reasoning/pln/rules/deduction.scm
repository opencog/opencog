; =============================================================================
; Deduction Rule
; TODO The rule must be applicable to ImplicationLink, SubsetLink and PartOfLink
;
; AND(Inheritance A B, Inheritance B C) entails Inheritance A C
; -----------------------------------------------------------------------------
(load "formulas.scm")
(define deduction-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
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
            (GroundedSchemaNode "scm: simple-deduction-formula")
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


; -----------------------------------------------------------------------------
; Deduction Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of AC may be updated
; -----------------------------------------------------------------------------
(define (simple-deduction-formula AB BC AC)
    (cog-set-tv!
        AC
        (simple-deduction-side-effect-free-formula AB BC AC)
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
(define (simple-deduction-side-effect-free-formula AB BC AC)
    (let
        ((sA (cog-stv-strength (gar AB)))
         (sB (cog-stv-strength (gdr AB)))
         (sC (cog-stv-strength (gdr BC)))
         (sAB (cog-stv-strength AB))
         (sBC (cog-stv-strength BC))
        )

        ; Returns sAC. Includes the consistency conditions
        (define (strength)
            (simple-deduction-strength-formula sA sB sC sAB sBC))

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

; Name the rule
(define deduction-rule-name (Node "deduction-rule"))
(DefineLink deduction-rule-name deduction-rule)
