;; =====================================================================
;; Body
;; |-
;; LambdaLink
;;    V
;;    Body
;;
;; Wrap a Lambda around a fully grounded body and assign to the lambda
;; the TV of the body. So basically it create a constant predicate or
;; schema.
;;
;; Note that in practice we actually match only premises already
;; wrapped in a LambdaLink as a workaround the current lack of
;; alpha-equivalence support.
;; ----------------------------------------------------------------------

(define lambda-fully-grounded-construction-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$V")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")
           (TypeNode "VariableNode")))
     (VariableNode "$B")))

(define lambda-fully-grounded-construction-body
  (AndLink
     (LambdaLink
        (VariableNode "$V")
        (VariableNode "$B"))
     (EvaluationLink
        (GroundedPredicateNode "scm: fully-grounded")
        (ListLink
           (VariableNode "$B")))))

(define lambda-fully-grounded-construction-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: lambda-fully-grounded-construction-formula")
     (ListLink
        (LambdaLink
           (VariableNode "$V")
           (VariableNode "$B")))))

(define lambda-fully-grounded-construction-rule
  (BindLink
     lambda-fully-grounded-construction-variables
     lambda-fully-grounded-construction-body
     lambda-fully-grounded-construction-rewrite))

(define (lambda-fully-grounded-construction-formula lamb)
  (let* ((lamb-outgoings (cog-outgoing-set lamb))
         (body (cadr lamb-outgoings))
         (body-tv (cog-tv body)))
    (cog-set-tv! lamb body-tv)))

(define (is-variable atom)
  (equal? (cog-type atom) 'VariableNode))

(define (rec-fully-grounded atom)
  (if (cog-node? atom)
      (not (is-variable atom))
      (every rec-fully-grounded (cog-outgoing-set atom))))

(define (fully-grounded atom)
  (if (rec-fully-grounded atom)
      (stv 1 1)
      (stv 0 1)))
