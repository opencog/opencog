;; =======================================================================
;; Implication Scope Factorization Rule
;; (TODO add wiki page)
;;
;; ImplicationLink
;;    LambdaLink
;;       V
;;       P
;;    LambdaLink
;;       V
;;       Q
;; |-
;; ImplicationScopeLink
;;    V
;;    P
;;    Q
;;
;; where V is a variable or a list of variables, P and Q are the
;; implicant and implicand bodies.
;; -----------------------------------------------------------------------

(define implication-scope-factorization-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-scope-factorization-body
  (ImplicationLink
     (QuoteLink (LambdaLink
        (UnquoteLink (VariableNode "$TyVs"))
        (UnquoteLink (VariableNode "$P"))))
     (QuoteLink (LambdaLink
        (UnquoteLink (VariableNode "$TyVs"))
        (UnquoteLink (VariableNode "$Q"))))))

(define implication-scope-factorization-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-scope-factorization-formula")
     (ListLink
        implication-scope-factorization-body
        (QuoteLink (ImplicationScopeLink
           (UnquoteLink (VariableNode "$TyVs"))
           (UnquoteLink (VariableNode "$P"))
           (UnquoteLink (VariableNode "$Q")))))))

(define implication-scope-factorization-rule
  (BindLink
     implication-scope-factorization-variables
     implication-scope-factorization-body
     implication-scope-factorization-rewrite))

(define (implication-scope-factorization-formula lamb-Impl Impl)
  (let* (
         (Impl-outgoings (cog-outgoing-set Impl))
         (SV (car Impl-outgoings))
         (P (cadr Impl-outgoings))
         (Q (caddr Impl-outgoings))
         (lamb-Impl-tv (cog-tv lamb-Impl)))
    (cog-set-tv!
     (ImplicationScopeLink
        SV
        P
        Q)
     lamb-Impl-tv)))

;; Name the rule
(define implication-scope-factorization-rule-name
  (DefinedSchemaNode "implication-scope-factorization-rule"))
(DefineLink implication-scope-factorization-rule-name
  implication-scope-factorization-rule)
