;; =====================================================================
;; Predicate <TV>
;; |-
;; Lambda <TV>
;;    <variables>
;;    Evaluation
;;       Predicate <TV>
;;       List
;;          <variables>
;;
;; Wrap a Lambda around an evaluation of predicate and assign to the
;; lambda the TV of the predicate.
;;
;; Note that in practice we actually match only predicates already
;; wrapped in a LambdaLink as a workaround the current lack of
;; predicate type definition and inference, and alpha-equivalence
;; support.
;; ----------------------------------------------------------------------

(define lambda-predicate-construction-variables
  (VariableList
     (TypedVariable
        (Variable "$V")
        (TypeChoice
           (Type "TypedVariableLink")
           (Type "VariableList")
           (Type "VariableNode")))
     (Variable "$P")
     (Variable "$Args")))

(define lambda-predicate-construction-body
  (Lambda
     (VariableNode "$V")
     (EvaluationLink
        (Variable "$P")
        (Variable "$Args"))))

(define lambda-predicate-construction-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: lambda-predicate-construction-formula")
     (ListLink
        (Variable "$P")
        lambda-predicate-construction-body)))

(define lambda-predicate-construction-rule
  (BindLink
     lambda-predicate-construction-variables
     lambda-predicate-construction-body
     lambda-predicate-construction-rewrite))

(define (lambda-predicate-construction-formula pred lamb)
  (let ((pred-tv (cog-tv pred))
        (pred-s (cog-stv-strength pred))
        (pred-c (cog-stv-confidence pred)))
    (if (= pred-c 0) ; Try to avoid constructing informationless
                     ; knowledge
        (cog-undefined-handle)
        (cog-merge-hi-conf-tv! lamb pred-tv))))

;; Name the rule
(define lambda-predicate-construction-rule-name
  (DefinedSchemaNode "lambda-predicate-construction-rule"))
(DefineLink lambda-predicate-construction-rule-name
  lambda-predicate-construction-rule)
