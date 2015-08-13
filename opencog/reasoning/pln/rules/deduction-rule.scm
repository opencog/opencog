; =============================================================================
; DeductionRule
; 
; AndLink
;   LinkType
;       A
;       B
;   LinkType
;       B
;       C
; |-
; LinkType
;   A
;   C
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-deduction
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
            (GroundedSchemaNode "scm: pln-formula-deduction")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (VariableNode "$D")
                (VariableNode "$E")))))

(define (pln-formula-deduction A B C AB BC)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sC (cog-stv-strength C))
         (cC (cog-stv-confidence C))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sBC (cog-stv-strength BC))
         (cBC (cog-stv-confidence BC)))
        (cond
            [(and
                (= (gar AB) A)
                (= (gdr AB) B)
                (= (gar BC) B)
                (= (gdr BC) C)
                (not (= (gar AB) (gdr BC)))
                (= (cog-type AB) (cog-type BC)))
             ((cog-type AB) (stv (simple-deduction-formula sA sB sC sAB sBC) (min cAB cBC))
                A C)
            ]
         )
    )
)
