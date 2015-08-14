; =============================================================================
; AbductionRule
; 
; AndLink
;   LinkType
;       A
;       B
;   LinkType
;       C
;       B
; |-
; LinkType
;   A
;   C
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-abduction
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
            (GroundedSchemaNode "scm: pln-formula-abduction")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (VariableNode "$D")
                (VariableNode "$E")))))

(define (pln-formula-abduction A B C AB CB)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sC (cog-stv-strength C))
         (cC (cog-stv-confidence C))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sCB (cog-stv-strength CB))
         (cCB (cog-stv-confidence CB)))
        (cond
            [(and
                (= (gar AB) A)
                (= (gdr AB) B)
                (= (gar CB) C)
                (= (gdr CB) B)
                (not (= (gar AB) (gar CB)))
                (= (cog-type AB) (cog-type CB)))
             ((cog-type AB) (stv (simple-deduction-formula sA sB sC sAB (inversion-formulat sCB sC sB)) (min cAB cCB))
                A C)
            ]
         )
    )
)
