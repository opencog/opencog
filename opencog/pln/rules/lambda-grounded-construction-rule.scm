;; =====================================================================
;; Body
;; |-
;; LambdaLink
;;    V
;;    Body
;;
;; Wrap a Lambda around a closed atom (that is an atom without free
;; variables in it) and assign to the lambda the TV of the body. So
;; basically it create a constant predicate or schema.
;;
;; Note that in practice we actually match only premises already
;; wrapped in a LambdaLink as a workaround the current lack of
;; alpha-equivalence support.
;; ----------------------------------------------------------------------

(define lambda-grounded-construction-vardecl
  (VariableList
     (TypedVariableLink
        (VariableNode "$V")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")
           (TypeNode "VariableNode")))
     (VariableNode "$B")))

(define lambda-grounded-construction-pattern
  (AndLink
     (VariableNode "$V")
     (VariableNode "$B")
     (EvaluationLink
        (GroundedPredicateNode "scm: lambda-grounded-construction-precondition")
        (VariableNode "$B"))))

(define lambda-grounded-construction-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: lambda-grounded-construction-formula")
     (ListLink
        (QuoteLink (LambdaLink
           (UnquoteLink (VariableNode "$V"))
           (UnquoteLink (VariableNode "$B"))))
        (VariableNode "$B"))))

(define lambda-grounded-construction-rule
  (BindLink
     lambda-grounded-construction-vardecl
     lambda-grounded-construction-pattern
     lambda-grounded-construction-rewrite))

(define (lambda-grounded-construction-formula lamb body)
  (cog-set-tv! lamb (cog-tv body)))

(define (lambda-grounded-construction-precondition atom)
  (bool->tv (and (cog-closed? atom) (tv-positive-conf? (cog-tv atom)))))

;; Name the rule
(define lambda-grounded-construction-rule-name
  (DefinedSchemaNode "lambda-grounded-construction-rule"))
(DefineLink lambda-grounded-construction-rule-name
  lambda-grounded-construction-rule)
