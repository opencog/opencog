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

(define lambda-grounded-construction-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$V")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")
           (TypeNode "VariableNode")))
     (VariableNode "$B")))

(define lambda-grounded-construction-body
  (AndLink
     (LambdaLink
        (VariableNode "$V")
        (VariableNode "$B"))
     (EvaluationLink
        (GroundedPredicateNode "scm: fully-grounded")
        (ListLink
           (VariableNode "$B")))))

(define lambda-grounded-construction-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: lambda-grounded-construction-formula")
     (ListLink
        (LambdaLink
           (VariableNode "$V")
           (VariableNode "$B")))))

(define lambda-grounded-construction-rule
  (BindLink
     lambda-grounded-construction-variables
     lambda-grounded-construction-body
     lambda-grounded-construction-rewrite))

(define (lambda-grounded-construction-formula lamb)
  (let* ((lamb-outgoings (cog-outgoing-set lamb))
         (body (cadr lamb-outgoings))
          (body-tv (cog-tv body)))
    (cog-set-tv! lamb body-tv)))

;; The following are for checking that an atom (i.e. a sub-hypergraph)
;; doesn't have free variables.
;;
;; TODO this should be replaced by a scheme binding from a C++
;; function, given how frequent its use should be.
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

;; Name the rule
(define lambda-grounded-construction-rule-name
  (DefinedSchemaNode "lambda-grounded-construction-rule"))
(DefineLink lambda-grounded-construction-rule-name
  lambda-grounded-construction-rule)
