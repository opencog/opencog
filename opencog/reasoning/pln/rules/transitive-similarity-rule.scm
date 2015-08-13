; =============================================================================
; TransitiveSimilarityRule
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

(define pln-rule-transitive-similarity
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (TypedVariableLink
                (VariableNode "$D")
                (TypeChoice
                    (TypeNode "SimilarityLink")
                    (TypeNode "ExtensionalSimilartiyLink")
                    (TypeNode "IntensionalSimilarityLink")))
            (TypedVariableLink
                (VariableNode "$E")
                (TypeChoice
                    (TypeNode "SimilarityLink")
                    (TypeNode "ExtensionalSimilarityLink")
                    (TypeNode "IntensionalSimilarityLink"))))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (VariableNode "$D")
            (VariableNode "$E"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-transitive-similarity")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (VariableNode "$D")
                (VariableNode "$E")))))

(define (pln-formula-transitive-similarity A B C AB BC)
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
             ((cog-type AB) (stv (transitive-similarity-formula sA sB sC sAB sBC) (min cAB cBC))
                A C)
            ]
         )
    )
)
