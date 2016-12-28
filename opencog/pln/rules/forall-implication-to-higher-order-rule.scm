;; =======================================================================
;; ForAll Implication to higher order rule
;; (TODO add wiki page)
;;
;; ForAllLink
;;    V
;;    ImplicationLink
;;       P[V]
;;       Q[V]
;; |-
;; ImplicationLink
;;    LambdaLink
;;       V
;;       P[V]
;;    LambdaLink
;;       V
;;       Q[V]
;;
;; where V is a variable, and P[V] and Q[V] are predicate bodies
;; containing V.
;; -----------------------------------------------------------------------

(use-modules (srfi srfi-1))

(use-modules (opencog exec))
(use-modules (opencog logger))

(define forall-implication-to-higher-order-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (TypedVariableLink        
        (VariableNode "$Impl")
        (TypeNode "ImplicationLink"))))

(define forall-implication-to-higher-order-body
  (QuoteLink (ForAllLink
     (UnquoteLink (VariableNode "$TyVs"))
     (UnquoteLink (VariableNode "$Impl")))))

(define forall-implication-to-higher-order-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: forall-implication-to-higher-order-formula")
     (ListLink
        (VariableNode "$TyVs")
        (VariableNode "$Impl"))))

(define forall-implication-to-higher-order-rule
  (BindLink
     forall-implication-to-higher-order-variables
     forall-implication-to-higher-order-body
     forall-implication-to-higher-order-rewrite))

;; TODO: only crisp for now
(define (forall-implication-to-higher-order-formula TyVs Impl)
  (cog-set-tv!
   (let* (
          (impl-outgoings (cog-outgoing-set Impl))
          (impl-t (car impl-outgoings))
          (impl-d (cadr impl-outgoings)))
     (ImplicationLink
         (LambdaLink
             TyVs
             impl-t)
         (LambdaLink
             TyVs
             impl-d)))
   (stv 1 1)))

;; Name the rule
(define forall-implication-to-higher-order-rule-name
  (DefinedSchemaNode "forall-implication-to-higher-order-rule"))
(DefineLink forall-implication-to-higher-order-rule-name
  forall-implication-to-higher-order-rule)
