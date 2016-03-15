;; =====================================================================
;; Implication implicant distribution rule
;;
;; ImplicationLink
;;    P
;;    Q
;; |-
;; ImplicationLink
;;    P
;;    AndLink
;;       P
;;       Q
;;
;;----------------------------------------------------------------------

(define implication-implicant-distribution-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$P")
        (TypeChoice
           (TypeNode "PredicateNode")
           (TypeNode "LambdaLink")))
     (TypedVariableLink
        (VariableNode "$Q")
        (TypeChoice
           (TypeNode "PredicateNode")
           (TypeNode "LambdaLink")))))

(define implication-implicant-distribution-body
  (ImplicationLink
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-implicant-distribution-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-implicant-distribution-formula")
     (ListLink
        implication-implicant-distribution-body)))

(define implication-implicant-distribution-rule
  (BindLink
     implication-implicant-distribution-variables
     implication-implicant-distribution-body
     implication-implicant-distribution-rewrite))

(define (implication-implicant-distribution-formula impl)
  (let* (
         (impl-outgoings (cog-outgoing-set impl))
         (P (car impl-outgoings))
         (Q (cadr impl-outgoings)))
    (cog-set-tv!
     (ImplicationLink
        P
        (cog-new-flattened-link 'AndLink P Q))
     (cog-tv impl))))

;; Name the rule
(define implication-implicant-distribution-rule-name
  (DefinedSchemaNode "implication-implicant-distribution-rule"))
(DefineLink implication-implicant-distribution-rule-name
  implication-implicant-distribution-rule)
