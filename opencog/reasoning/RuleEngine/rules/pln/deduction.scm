; =============================================================================
; Deduction Rule
; TODO The rule must be applicable to ImplicationLink, SubsetLink and PartOfLink
;
; AND(Inheritance A B, Inheritance B C) entails Inheritance A C
; -----------------------------------------------------------------------------

(define pln-rule-deduction
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$A")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$B")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$C")
                (TypeNode "ConceptNode")
            )
        )
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
                ; To avoid (Inheritance A B)  (Inheritance B A)
                #!(AbsentLink
                    (EvaluationLink
                        (GroundedPredicateNode "scm: cog-equal?")
                        (ListLink
                            (VariableNode "$A")
                            (VariableNode "$B")
                        )
                    )
                )!#
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
; Check whether two nodes are equal.
; -----------------------------------------------------------------------------
(define (cog-equal? atom-1 atom-2)
    (if (equal? atom-1 atom-2)
        (stv 1 1)
        (stv 0 1)
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
            (cond
                (
                    (and
                        (<=
                            (max (/ (- (+ sA sB) 1) sA) 0)
                            sAB
                            (min 1 (/ sB sA))
                        )
                        (<=
                            (max (/ (- (+ sB sC) 1) sB) 0)
                            sBC
                            (min 1 (/ sC sB))
                        )
                    )
                    (+
                        (* sAB sBC)
                        (/
                            (* (- 1 sAB) (- sC (* sB sBC)))
                            (- 1 sB)
                        )
                    )
                )
                (
                    (and (= 1 sB) (= sC sBC))
                    (* sAB sBC)
                )
                (else 0)
            )
        )

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
