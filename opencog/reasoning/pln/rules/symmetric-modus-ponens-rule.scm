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
; Due to issues in pattern matching in backward chaining, the files has been
; split into three rules for seperate link types. The 3 rules are
;           symmetric-modus-ponens-similarity-rule
;           symmetric-modus-ponens-intensional-similarity-rule
;           summetric-modus-ponens-extensional-similarity-rule
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define symmetric-modus-ponens-similarity-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (SimilarityLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: symmetric-modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (SimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define symmetric-modus-ponens-intensional-similarity-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (IntensionalSimilarityLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: symmetric-modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (IntensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define symmetric-modus-ponens-extensional-similarity-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (ExtensionalSimilarityLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: symmetric-modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (ExtensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define (symmetric-modus-ponens-formula A AB B)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (snotAB 0.2)
         (cnotAB 1))
        (cog-set-tv! 
            B 
            (stv 
                (+ (* sA sAB) (* (* snotAB (negate sA)) (+ 1 sAB))) 
                (min (min cAB cnotAB) cA)))))
