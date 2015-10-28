;; =============================================================================
;; ModusPonensRule
;; 
;; <LinkType>
;;   A
;;   B
;; A
;; |-
;; B
;;
;; Due to type system limitations, the rule has been divided into 3:
;;       modus-ponens-inheritance-rule
;;       modus-ponens-implication-rule
;;       modus-ponens-subset-rule
;;
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

;; Generate the corresponding modus ponens rule given its link-type.
(define (gen-modus-ponens-rule link-type)
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (link-type
            (VariableNode "$A")
            (VariableNode "$B"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define modus-ponens-inheritance-rule
  (gen-modus-ponens-rule InheritanceLink))

(define modus-ponens-implication-rule
  (gen-modus-ponens-rule ImplicationLink))

(define modus-ponens-subset-rule
  (gen-modus-ponens-rule SubsetLink))

(define (modus-ponens-formula A AB B)
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
                (precise-modus-ponens-strength-formula sA sAB snotAB) 
                (min (min cAB cnotAB) cA)))))

