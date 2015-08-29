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
;           pln-rule-symmetric-modus-ponens-similarity
;           pln-rule-symmetric-modus-ponens-intensional-similarity
;           pln-rule-summetric-modus-ponens-extensional-similarity
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-symmetric-modus-ponens-similarity
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (SimilarityLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-symmetric-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (SimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define pln-rule-symmetric-modus-ponens-intensional-similarity
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (IntensionalSimilarityLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-symmetric-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (IntensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define pln-rule-symmetric-modus-ponens-extensional-similarity
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (ExtensionalSimilarityLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-symmetric-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (ExtensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define (pln-formula-symmetric-modus-ponens A AB B)
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
