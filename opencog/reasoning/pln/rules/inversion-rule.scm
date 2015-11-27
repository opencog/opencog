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
                (VariableNode "$A")
                (VariableNode "$B")
                (link-type
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (link-type
                    (VariableNode "$B")
                    (VariableNode "$A"))))))

(define inversion-inheritance-rule
    (gen-inversion-rule InheritanceLink))

(define inversion-implication-rule
    (gen-inversion-rule ImplicationLink))

(define inversion-subset-rule
    (gen-inversion-rule SubsetLink))

(define (inversion-formula A B AB BA)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB)))
        (cog-set-tv!
            BA
            (stv 
                (/ (* sAB sB) (floor sA)) 
                (min sA sB sAB)))))

;; Name the rules
(define inversion-inheritance-rule-name
  (Node "inversion-inheritance-rule"))
(DefineLink inversion-inheritance-rule-name
  inversion-inheritance-rule)

(define inversion-implication-rule-name
  (Node "inversion-implication-rule"))
(DefineLink inversion-implication-rule-name
  inversion-implication-rule)

(define inversion-subset-rule-name
  (Node "inversion-subset-rule"))
(DefineLink inversion-subset-rule-name
  inversion-subset-rule)
