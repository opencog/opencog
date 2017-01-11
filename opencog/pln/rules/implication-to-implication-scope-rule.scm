;; =======================================================================
;; Implication Scope to Implication Rule
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

(define implication-to-implication-scope-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-to-implication-scope-body
  (ImplicationLink
     (QuoteLink (LambdaLink
        (UnquoteLink (VariableNode "$TyVs"))
        (UnquoteLink (VariableNode "$P"))))
     (QuoteLink (LambdaLink
        (UnquoteLink (VariableNode "$TyVs"))
        (UnquoteLink (VariableNode "$Q"))))))

(define implication-to-implication-scope-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-to-implication-scope-formula")
     (ListLink
        implication-to-implication-scope-body
        (QuoteLink (ImplicationScopeLink
           (UnquoteLink (VariableNode "$TyVs"))
           (UnquoteLink (VariableNode "$P"))
           (UnquoteLink (VariableNode "$Q")))))))

(define implication-to-implication-scope-rule
  (BindLink
     implication-to-implication-scope-variables
     implication-to-implication-scope-body
     implication-to-implication-scope-rewrite))

(define (implication-to-implication-scope-formula lamb-Impl Impl)
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
(define implication-to-implication-scope-rule-name
  (DefinedSchemaNode "implication-to-implication-scope-rule"))
(DefineLink implication-to-implication-scope-rule-name
  implication-to-implication-scope-rule)
