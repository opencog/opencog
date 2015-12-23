;; =============================================================================
;; Term Probability Rule
;;
;; <LinkType>
;;   A
;;   B
;; <LinkType>
;;   B
;;   A
;; A
;; |-
;; B
;;
;; Due to type system limitations, the rule has been divided into 3:
;;           term-probability-inheritance-rule
;;           term-probability-implication-rule
;;           term-probability-subset-rule
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

;; Generate the corresponding term probability rule given its
;; link-type.
(define (gen-term-probability-rule link-type)
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (link-type
                (VariableNode "$A")
                (VariableNode "$B"))
            (link-type
                (VariableNode "$B")
                (VariableNode "$A")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: term-probability-formula")
            (ListLink
                (VariableNode "$A")
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (link-type
                    (VariableNode "$B")
                    (VariableNode "$A"))
                (VariableNode "$B")))))

(define term-probability-inheritance-rule
  (gen-term-probability-rule InheritanceLink))

(define term-probability-implication-rule
  (gen-term-probability-rule ImplicationLink))

(define term-probability-subset-rule
  (gen-term-probability-rule SubsetLink))

(define (term-probability-formula A AB BA B)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sBA (cog-stv-strength BA))
         (cBA (cog-stv-confidence BA)))
        (cog-set-tv! 
            B 
            (stv 
                (/ (* sA sAB) sBA) 
                (min (min cAB cBA) cA)))))
