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

(define implication-implicant-distribution-vardecl
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

(define implication-implicant-distribution-pattern
  (And
     (ImplicationLink
        (VariableNode "$P")
        (VariableNode "$Q"))
     (EvaluationLink
        (GroundedPredicateNode "scm: gt-zero-confidence")
        (ImplicationLink
           (VariableNode "$P")
           (VariableNode "$Q")))))

(define implication-implicant-distribution-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-implicant-distribution-formula")
     (ListLink
        (ImplicationLink
           (VariableNode "$P")
           (And
              (VariableNode "$P")
              (VariableNode "$Q")))
        (ImplicationLink
           (VariableNode "$P")
           (VariableNode "$Q")))))

(define implication-implicant-distribution-rule
  (BindLink
     implication-implicant-distribution-vardecl
     implication-implicant-distribution-pattern
     implication-implicant-distribution-rewrite))

(define (implication-implicant-distribution-formula DImpl Impl)
  (cog-merge-hi-conf-tv! DImpl (cog-tv Impl)))

;; Name the rule
(define implication-implicant-distribution-rule-name
  (DefinedSchemaNode "implication-implicant-distribution-rule"))
(DefineLink implication-implicant-distribution-rule-name
  implication-implicant-distribution-rule)
