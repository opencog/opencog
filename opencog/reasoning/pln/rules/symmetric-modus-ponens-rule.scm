;; =============================================================================
;; SymmetricModusPonensRule
;;
;; <LinkType>
;;   A
;;   B
;; A
;; |-
;; B
;;
;; Due to type system limitations, the rule has been divided into 3:
;;           symmetric-modus-ponens-similarity-rule
;;           symmetric-modus-ponens-intensional-similarity-rule
;;           summetric-modus-ponens-extensional-similarity-rule
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

;; Generate the corresponding symmetric modus ponens rule given its
;; link-type.
(define (gen-symmetric-modus-ponens-rule link-type)
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (link-type
            (VariableNode "$A")
            (VariableNode "$B"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: symmetric-modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define symmetric-modus-ponens-similarity-rule
  (gen-symmetric-modus-ponens-rule SimilarityLink))

(define symmetric-modus-ponens-intensional-similarity-rule
  (gen-symmetric-modus-ponens-rule IntensionalSimilarityLink))

(define symmetric-modus-ponens-extensional-similarity-rule
  (gen-symmetric-modus-ponens-rule ExtensionalSimilarityLink))

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
