;; =============================================================================
;; TransitiveSimilarityRule
;;
;; <LinkType>
;;   A
;;   B
;; <LinkType>
;;   B
;;   C
;; |-
;; <LinkType>
;;   A
;;   C
;;
;; Due to type system limitations, the rule has been divided into 3:
;;           transitive-similarity-similarity-rule
;;           transitive-similarity-extensional-similarity-rule
;;           transitive-similarity-intensional-similarity-rule
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

(define (gen-transitive-similarity-rule link-type)
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (link-type
                (VariableNode "$A")
                (VariableNode "$B"))
            (link-type
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: transitive-similarity-formula")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (link-type
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$C"))))))

(define transitive-similarity-similarity-rule
  (gen-transitive-similarity-rule SimilarityLink))

(define transitive-similarity-extensional-similarity-rule
  (gen-transitive-similarity-rule ExtensionalSimilarityLink))

(define transitive-similarity-intensional-similarity-rule
  (gen-transitive-similarity-rule IntensionalSimilarityLink))

(define (transitive-similarity-formula A B C AB BC AC)
    (let
        ((sA (cog-mean A))
         (cA (cog-confidence A))
         (sB (cog-mean B))
         (cB (cog-confidence B))
         (sC (cog-mean C))
         (cC (cog-confidence C))
         (sAB (cog-mean AB))
         (cAB (cog-confidence AB))
         (sBC (cog-mean BC))
         (cBC (cog-confidence BC)))
        (cog-set-tv!
            AC 
            (stv 
                (transitive-similarity-strength-formula sA sB sC sAB sBC) 
                (min cAB cBC)))))
