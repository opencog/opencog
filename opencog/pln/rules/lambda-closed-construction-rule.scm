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

(define lambda-closed-construction-vardecl
  (VariableList
     (TypedVariableLink
        (VariableNode "$V")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")
           (TypeNode "VariableNode")))
     (VariableNode "$B")))

(define lambda-closed-construction-pattern
  (AndLink
     (VariableNode "$V")
     (VariableNode "$B")
     (EvaluationLink
        (GroundedPredicateNode "scm: lambda-closed-construction-precondition")
        (VariableNode "$B"))))

(define lambda-closed-construction-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: lambda-closed-construction-formula")
     (ListLink
        (QuoteLink (LambdaLink
           (UnquoteLink (VariableNode "$V"))
           (UnquoteLink (VariableNode "$B"))))
        (VariableNode "$B"))))

(define lambda-closed-construction-rule
  (BindLink
     lambda-closed-construction-vardecl
     lambda-closed-construction-pattern
     lambda-closed-construction-rewrite))

(define (lambda-closed-construction-formula lamb body)
  (cog-set-tv! lamb (cog-tv body)))

(define (lambda-closed-construction-precondition atom)
  (bool->tv (and (cog-closed? atom) (tv-positive-conf? (cog-tv atom)))))

;; Name the rule
(define lambda-closed-construction-rule-name
  (DefinedSchemaNode "lambda-closed-construction-rule"))
(DefineLink lambda-closed-construction-rule-name
  lambda-closed-construction-rule)
