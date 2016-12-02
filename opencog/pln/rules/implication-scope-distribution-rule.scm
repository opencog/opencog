;; =======================================================================
;; Implication Scope Distribution Rule
;; (TODO add wiki page)
;;
;; ImplicationScopeLink
;;    V
;;    P
;;    Q
;; |-
;; ImplicationLink
;;    LambdaLink
;;       V
;;       P
;;    LambdaLink
;;       V
;;       Q
;;
;; where V is a variable or a list of variables, P and Q are the
;; implicant and implicand bodies.
;; -----------------------------------------------------------------------

(define implication-scope-distribution-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-scope-distribution-body
  (Quote (ImplicationScopeLink
     (Unquote (VariableNode "$TyVs"))
     (Unquote (VariableNode "$P"))
     (Unquote (VariableNode "$Q")))))

(define implication-scope-distribution-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-scope-distribution-formula")
     (ListLink
        implication-scope-distribution-body
        (Implication
           (LocalQuote
           (Lambda
              (VariableNode "$TyVs")
              (VariableNode "$P")))
           (LocalQuote
           (Lambda
              (VariableNode "$TyVs")
              (VariableNode "$Q")))))))

(define implication-scope-distribution-rule
  (BindLink
     implication-scope-distribution-variables
     implication-scope-distribution-body
     implication-scope-distribution-rewrite))

(define (implication-scope-distribution-formula ImplSc Impl)
  (cog-set-tv! Impl (cog-tv ImplSc)))

;; Name the rule
(define implication-scope-distribution-rule-name
  (DefinedSchemaNode "implication-scope-distribution-rule"))
(DefineLink implication-scope-distribution-rule-name
  implication-scope-distribution-rule)
