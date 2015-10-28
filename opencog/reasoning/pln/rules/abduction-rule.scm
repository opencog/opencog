;; =============================================================================
;; AbductionRule
;; 
;; <LinkType>
;;   A
;;   B
;; <LinkType>
;;   C
;;   B
;; |-
;; <LinkType>
;;   A
;;   C
;;
;; Due to type system limitations, the rule has been divided into 3:
;;       abduction-inheritance-rule
;;       abduction-implication-rule
;;       abduction-subset-rule
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

;; Generate the corresponding abduction rule given its link-type.
(define (gen-abduction-rule link-type)
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
                (VariableNode "$C")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: abduction-formula")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (link-type
                    (VariableNode "$C")
                    (VariableNode "$B"))
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$C"))))))

(define abduction-inheritance-rule
  (gen-abduction-rule InheritanceLink))

(define abduction-implication-rule
  (gen-abduction-rule ImplicationLink))

(define abduction-subset-rule
  (gen-abduction-rule SubsetLink))

(define (abduction-formula A B C AB CB AC)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sC (cog-stv-strength C))
         (cC (cog-stv-confidence C))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sCB (cog-stv-strength CB))
         (cCB (cog-stv-confidence CB)))
        (cog-set-tv! 
            AC
            (stv 
                (simple-deduction-strength formula sA sB sC sAB
                                           (inversion-strength-formula sCB sC sB))
                (min cAB cCB)))))

;; Name the rules
(define abduction-inheritance-rule-name
  (Node "abduction-inheritance-rule"))
(DefineLink abduction-inheritance-rule-name
  abduction-inheritance-rule)

(define abduction-implication-rule-name
  (Node "abduction-implication-rule"))
(DefineLink abduction-implication-rule-name
  abduction-implication-rule)

(define abduction-subset-rule-name
  (Node "abduction-subset-rule"))
(DefineLink abduction-subset-rule-name
  abduction-subset-rule)
