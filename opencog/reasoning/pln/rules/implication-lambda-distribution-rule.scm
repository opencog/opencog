;; =======================================================================
;; Implication Lambda Distribution Rule
;; (TODO add wiki page)
;;
;; ImplicationLink
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

(define implication-lambda-distribution-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-lambda-distribution-body
  (ImplicationLink
     (VariableNode "$TyVs")
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-lambda-distribution-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-lambda-distribution-formula")
     (ListLink
        implication-lambda-distribution-body)))

(define implication-lambda-distribution-rule
  (BindLink
     implication-lambda-distribution-variables
     implication-lambda-distribution-body
     implication-lambda-distribution-rewrite))

(define (implication-lambda-distribution-formula Impl)
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
