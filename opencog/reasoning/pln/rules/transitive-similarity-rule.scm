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
; Due to issues in pattern matching in backward chaining, the files has been
; split into three rules for seperate link types. The 3 rules are
;           transitive-similarity-similarity-rule
;           transitive-similarity-extensional-similarity-rule
;           transitive-similarity-intensional-similarity-rule
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define transitive-similarity-similarity-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (SimilarityLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (SimilarityLink
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: transitive-similarity-formula")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (SimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SimilarityLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (SimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$C"))))))

(define transitive-similarity-extensional-similarity-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (ExtensionalSimilarityLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (ExtensionalSimilarityLink
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: transitive-similarity-formula")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (ExtensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ExtensionalSimilarityLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (ExtensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$C"))))))

(define transitive-similarity-intensional-similarity-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (IntensionalSimilarityLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (IntensionalSimilarityLink
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: transitive-similarity-formula")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (IntensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (IntensionalSimilarityLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (IntensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$C"))))))

(define (transitive-similarity-formula A B C AB BC AC)
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
        (cog-set-tv!
            AC 
            (stv 
                (transitive-similarity-strength-formula sA sB sC sAB sBC) 
                (min cAB cBC)))))
