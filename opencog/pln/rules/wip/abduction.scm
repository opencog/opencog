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
;; TODO: make BC compatible (check if the premises could be unordered)
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
                (VariableNode "$B"))
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$C"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: abduction-formula")
            (ListLink
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

(define (abduction-formula AB CB AC)
    (let
        ((sA (cog-mean (gar AB)))
         (cA (cog-confidence (gar AB)))
         (sB (cog-mean (gdr AB)))
         (cB (cog-confidence (gdr AB)))
         (sC (cog-mean (gar CB)))
         (cC (cog-confidence (gar CB)))
         (sAB (cog-mean AB))
         (cAB (cog-confidence AB))
         (sCB (cog-mean CB))
         (cCB (cog-confidence CB)))
        (cog-merge-hi-conf-tv!
            AC
            (stv
                (simple-deduction-strength-formula sA sB sC sAB
                                           (inversion-strength-formula sCB sC sB))
                (min cAB cCB)))))

;; Name the rules
(define abduction-inheritance-rule-name
  (DefinedSchemaNode "abduction-inheritance-rule"))
(DefineLink abduction-inheritance-rule-name
  abduction-inheritance-rule)

(define abduction-implication-rule-name
  (DefinedSchemaNode "abduction-implication-rule"))
(DefineLink abduction-implication-rule-name
  abduction-implication-rule)

(define abduction-subset-rule-name
  (DefinedSchemaNode "abduction-subset-rule"))
(DefineLink abduction-subset-rule-name
  abduction-subset-rule)
