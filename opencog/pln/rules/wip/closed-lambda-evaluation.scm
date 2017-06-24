;; =====================================================================
;; Closed lambda evaluation rule
;;
;; LambdaLink
;;    V
;;    Body
;; |-
;; LambdaLink <TV>
;;    V
;;    Body
;;
;; Like closed-lambda-introduction-rule but merely calculate the TV of
;; an existing lambda link over a closed term instead of building
;; one. It is a strict refinement used to not pollute the atomspace
;; with extranous combinations when applied in a naive forward way.
;; ----------------------------------------------------------------------

(define closed-lambda-evaluation-vardecl
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

(define closed-lambda-evaluation-pattern
  (AndLink
    (QuoteLink (LambdaLink
      (UnquoteLink (VariableNode "$V"))
      (UnquoteLink (VariableNode "$B"))))
    (EvaluationLink
      (GroundedPredicateNode "scm: closed-lambda-evaluation-precondition")
      (VariableNode "$B"))))

(define closed-lambda-evaluation-rewrite
  (ExecutionOutputLink
    (GroundedSchemaNode "scm: closed-lambda-evaluation-formula")
    (ListLink
      (QuoteLink (LambdaLink
        (UnquoteLink (VariableNode "$V"))
        (UnquoteLink (VariableNode "$B"))))
      (VariableNode "$B"))))

(define closed-lambda-evaluation-rule
  (BindLink
    closed-lambda-evaluation-vardecl
    closed-lambda-evaluation-pattern
    closed-lambda-evaluation-rewrite))

(define (closed-lambda-evaluation-formula lamb body)
  (cog-set-tv! lamb (cog-tv body)))

(define (closed-lambda-evaluation-precondition atom)
  (bool->tv (and (cog-closed? atom) (tv-non-null-conf? (cog-tv atom)))))

;; Name the rule
(define closed-lambda-evaluation-rule-name
  (DefinedSchemaNode "closed-lambda-evaluation-rule"))
(DefineLink closed-lambda-evaluation-rule-name
  closed-lambda-evaluation-rule)
