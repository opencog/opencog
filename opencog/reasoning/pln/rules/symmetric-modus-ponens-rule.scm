; =============================================================================
; SymmetricModusPonensRule
; 
; AndLink
;   LinkType
;       A
;       B
;   A
; |-
;   B
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-symmetric-modus-ponens
    (BindLink
        (VariableList
            (VariableNode "$A")
            (TypedVariableLink
                (VariableNode "$C")
                (TypeChoice
                    (TypeNode "SimilarityLink")
                    (TypeNode "IntensionalSimilarityLink")
                    (TypeNode "ExtensionalSimilarityLink"))))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$C"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-symmetric-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$C")))))

(define (pln-formula-symmetric-modus-ponens A AB)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (snotAB 0.2)
         (cnotAB 1))
        (cond
            [(and
                (= (gar AB) A))
             (cog-set-tv! (gdr AB) (stv (+ (* sA sAB) (* (* snotAB (negate sA)) (+ 1 sAB))) (min (min cAB cnotAB) cA)))
            ]
         )
    )
)

