;; =====================================================================
;; LambdaLink
;;    V
;;    Body
;; |-
;; LambdaLink <TV>
;;    V
;;    Body
;;
;; Like lambda-closed-construction-rule but merely calculate the TV of
;; an existing lambda link over a closed term instead of building
;; one. It is a strict refinement used to not pollute the atomspace
;; with extranous combinations when applied in a naive forward way.
;; ----------------------------------------------------------------------

(define lambda-closed-evaluation-vardecl
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

(define lambda-closed-evaluation-pattern
  (AndLink
    (QuoteLink (LambdaLink
      (UnquoteLink (VariableNode "$V"))
      (UnquoteLink (VariableNode "$B"))))
    (EvaluationLink
      (GroundedPredicateNode "scm: lambda-closed-evaluation-precondition")
      (VariableNode "$B"))))

(define lambda-closed-evaluation-rewrite
  (ExecutionOutputLink
    (GroundedSchemaNode "scm: lambda-closed-evaluation-formula")
    (ListLink
      (QuoteLink (LambdaLink
        (UnquoteLink (VariableNode "$V"))
        (UnquoteLink (VariableNode "$B"))))
      (VariableNode "$B"))))

(define lambda-closed-evaluation-rule
  (BindLink
    lambda-closed-evaluation-vardecl
    lambda-closed-evaluation-pattern
    lambda-closed-evaluation-rewrite))

(define (lambda-closed-evaluation-formula lamb body)
  (cog-set-tv! lamb (cog-tv body)))

(define (lambda-closed-evaluation-precondition atom)
  (bool->tv (and (cog-closed? atom) (tv-non-null-conf? (cog-tv atom)))))

;; Name the rule
(define lambda-closed-evaluation-rule-name
  (DefinedSchemaNode "lambda-closed-evaluation-rule"))
(DefineLink lambda-closed-evaluation-rule-name
  lambda-closed-evaluation-rule)
