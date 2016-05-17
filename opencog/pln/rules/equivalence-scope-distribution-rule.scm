;; =======================================================================
;; Equivalence Scope Distribution Rule
;; (TODO add wiki page)
;;
;; EquivalenceLink
;;    V
;;    P
;;    Q
;; |-
;; EquivalenceLink
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

(define equivalence-scope-distribution-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define equivalence-scope-distribution-body
  (EquivalenceLink
     (VariableNode "$TyVs")
     (VariableNode "$P")
     (VariableNode "$Q")))

(define equivalence-scope-distribution-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: equivalence-scope-distribution-formula")
     (ListLink
        equivalence-scope-distribution-body)))

(define equivalence-scope-distribution-rule
  (BindLink
     equivalence-scope-distribution-variables
     equivalence-scope-distribution-body
     equivalence-scope-distribution-rewrite))

(define (equivalence-scope-distribution-formula Impl)
  (let* (
         (Impl-outgoings (cog-outgoing-set Impl))
         (SV (car Impl-outgoings))
         (P (cadr Impl-outgoings))
         (Q (caddr Impl-outgoings))
         (Impl-tv (cog-tv Impl)))
    (cog-set-tv!
     (EquivalenceLink
        (LambdaLink SV P)
        (LambdaLink SV Q))
     Impl-tv)))

;; Name the rule
(define equivalence-scope-distribution-rule-name
  (DefinedSchemaNode "equivalence-scope-distribution-rule"))
(DefineLink equivalence-scope-distribution-rule-name
  equivalence-scope-distribution-rule)
