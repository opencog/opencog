; =============================================================================
; InductionRule
; 
; AndLink
;   LinkType
;       A
;       B
;   LinkType
;       A
;       C
; |-
; LinkType
;   B
;   C
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-induction
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (TypedVariableLink
                (VariableNode "$D")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink")))
            (TypedVariableLink
                (VariableNode "$E")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink"))))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (VariableNode "$D")
            (VariableNode "$E"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-induction")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (VariableNode "$D")
                (VariableNode "$E")))))

(define (pln-formula-induction A B C AB AC)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sC (cog-stv-strength C))
         (cC (cog-stv-confidence C))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sAC (cog-stv-strength AC))
         (cAC (cog-stv-confidence AC)))
        (cond
            [(and
                (= (gar AB) A)
                (= (gdr AB) B)
                (= (gar AC) A)
                (= (gdr AC) C)
                (not (= (gdr AB) (gdr AC)))
                (= (cog-type AB) (cog-type AC)))
             ((cog-type AB) (stv (simple-deduction-formula sB sA sC (inversion-formulat sAB sA sB) sAC) (min cAB cAC))
                B C)
            ]
         )
    )
)
