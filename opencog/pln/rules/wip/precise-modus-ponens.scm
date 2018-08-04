;; =============================================================================
;; PreciseModusPonensRule
;; 
;; <LinkType>
;;   A
;;   B
;; <LinkType>
;;   NotLink
;;     A
;;   B
;; A
;; |-
;;   B
;; Due to type system limitations, the rule has been divided into 3:
;;       precise-modus-ponens-inheritance-rule
;;       precise-modus-ponens-implication-rule
;;       precise-modus-ponens-subset-rule
;;

; -----------------------------------------------------------------------------
(load "formulas.scm")

;; Generate the corresponding precise modus ponens rule given its
;; link-type.
(define (gen-precise-modus-ponens-rule link-type)
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (link-type
                (VariableNode "$A")
                (VariableNode "$B"))
            (link-type
                (NotLink
                    (VariableNode "$A"))
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: precise-modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (link-type
                    (NotLink
                        (VariableNode "$A"))
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define precise-modus-ponens-inheritance-rule
  (gen-precise-modus-ponens-rule InheritanceLink))

(define precise-modus-ponens-implication-rule
  (gen-precise-modus-ponens-rule ImplicationLink))

(define precise-modus-ponens-subset-rule
  (gen-precise-modus-ponens-rule SubsetLink))

(define (precise-modus-ponens-formula A AB notAB B)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (snotAB (cog-stv-strength notAB))
         (cnotAB (cog-stv-confidence notAB)))
        (cog-set-tv!
            B
            (stv 
                (precise-modus-ponens-strength-formula sA sAB snotAB) 
                (min (min cAB cnotAB) cA)))))
