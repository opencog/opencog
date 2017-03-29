;; =============================================================================
;; InversionRule
;;
;; <LinkType>
;;   A
;;   B
;; |-
;; <LinkType>
;;   B
;;   A
;;
;; Due to type system limitations, the rule has been divided into 3:
;;       inversion-inheritance-rule
;;       inversion-implication-rule
;;       inversion-subset-rule
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

;; Generate the corresponding inversion rule given its link-type.
(define (gen-inversion-rule link-type)
  (BindLink
    (VariableList
      (VariableNode "$A")
      (VariableNode "$B"))
    (link-type
      (VariableNode "$A")
      (VariableNode "$B"))
    (ExecutionOutputLink
      (GroundedSchemaNode "scm: inversion-formula")
      (ListLink
        (link-type
          (VariableNode "$B")
          (VariableNode "$A"))
        (VariableNode "$A")
        (VariableNode "$B")
        (link-type
          (VariableNode "$A")
          (VariableNode "$B"))))))

(define inversion-inheritance-rule
    (gen-inversion-rule InheritanceLink))

(define inversion-implication-rule
    (gen-inversion-rule ImplicationLink))

(define inversion-subset-rule
    (gen-inversion-rule SubsetLink))

(define (inversion-formula conclusion . premises)
  (if (= (length premises) 3)
    (let*
      ((BA conclusion)
       (A (list-ref premises 0))
       (B (list-ref premises 1))
       (AB (list-ref premises 2))
       (sA (cog-stv-strength A))
       (cA (cog-stv-confidence A))
       (sB (cog-stv-strength B))
       (cB (cog-stv-confidence B))
       (sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB))
       (sBA (inversion-strength-formula sA sB sAB))
       (cBA (* 0.9 (min cA cB cAB))))
    (if (and (< 1e-8 sBA) (< 1e-8 cBA) ; Try to avoid constructing
                                        ; informationless knowledge
             (inversion-consistency sA sB sAB))
        (cog-merge-hi-conf-tv! BA (stv sBA cBA))
        (cog-undefined-handle)))))

;; Name the rules
(define inversion-inheritance-rule-name
  (DefinedSchemaNode "inversion-inheritance-rule"))
(DefineLink inversion-inheritance-rule-name
  inversion-inheritance-rule)

(define inversion-implication-rule-name
  (DefinedSchemaNode "inversion-implication-rule"))
(DefineLink inversion-implication-rule-name
  inversion-implication-rule)

(define inversion-subset-rule-name
  (DefinedSchemaNode "inversion-subset-rule"))
(DefineLink inversion-subset-rule-name
  inversion-subset-rule)
