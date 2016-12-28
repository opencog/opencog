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
  (QuoteLink (ImplicationScopeLink
     (UnquoteLink (VariableNode "$TyVs"))
     (UnquoteLink (VariableNode "$P"))
     (UnquoteLink (VariableNode "$Q")))))

(define implication-scope-distribution-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-scope-distribution-formula")
     (ListLink
        implication-scope-distribution-body)))

(define implication-scope-distribution-rule
  (BindLink
     implication-scope-distribution-variables
     implication-scope-distribution-body
     implication-scope-distribution-rewrite))

(define (implication-scope-distribution-formula Impl)
  (let* (
         (Impl-outgoings (cog-outgoing-set Impl))
         (SV (car Impl-outgoings))
         (P (cadr Impl-outgoings))
         (Q (caddr Impl-outgoings))
         (Impl-tv (cog-tv Impl)))
    (cog-set-tv!
     (ImplicationLink
        (LambdaLink SV P)
        (LambdaLink SV Q))
     Impl-tv)))

;; Name the rule
(define implication-scope-distribution-rule-name
  (DefinedSchemaNode "implication-scope-distribution-rule"))
(DefineLink implication-scope-distribution-rule-name
  implication-scope-distribution-rule)
