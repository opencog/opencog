;; =====================================================================
;; Closed lamdba introduction rule
;;
;; V
;; Body
;; |-
;; LambdaLink
;;    V
;;    Body
;;
;; Wrap a Lambda around a closed atom (that is an atom without free
;; variables in it) and assign to the lambda the TV of the body. So
;; basically create a constant predicate or schema.
;; ----------------------------------------------------------------------

(define closed-lambda-introduction-vardecl
  (VariableList
    (TypedVariableLink
      (VariableNode "$V")
      (TypeChoice
        (TypeNode "TypedVariableLink")
        (TypeNode "VariableList")
        (TypeNode "VariableNode")))
    (TypedVariableLink
      (VariableNode "$B")
      (TypeNode "EvaluationLink"))))

(define closed-lambda-introduction-pattern
  (AndLink
    (VariableNode "$V")
    (VariableNode "$B")
    (EvaluationLink
      (GroundedPredicateNode "scm: closed-lambda-introduction-precondition")
      (VariableNode "$B"))))

(define closed-lambda-introduction-rewrite
  (ExecutionOutputLink
    (GroundedSchemaNode "scm: closed-lambda-introduction-formula")
    (ListLink
      (QuoteLink (LambdaLink
        (UnquoteLink (VariableNode "$V"))
        (UnquoteLink (VariableNode "$B"))))
      (VariableNode "$B"))))

(define closed-lambda-introduction-rule
  (BindLink
    closed-lambda-introduction-vardecl
    closed-lambda-introduction-pattern
    closed-lambda-introduction-rewrite))

(define (closed-lambda-introduction-formula lamb body)
  (cog-set-tv! lamb (cog-tv body)))

(define (closed-lambda-introduction-precondition atom)
  (bool->tv (and (cog-closed? atom) (tv-non-null-conf? (cog-tv atom)))))

;; Name the rule
(define closed-lambda-introduction-rule-name
  (DefinedSchemaNode "closed-lambda-introduction-rule"))
(DefineLink closed-lambda-introduction-rule-name
  closed-lambda-introduction-rule)
