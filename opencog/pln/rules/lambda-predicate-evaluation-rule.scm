;; =====================================================================
;; Lambda
;;    <variables>
;;    Evaluation
;;       Predicate <TV>
;;       List
;;          <variables>
;; |-
;; Lambda <TV>
;;    <variables>
;;    Evaluation
;;       Predicate <TV>
;;       List
;;          <variables>
;;
;; Evaluate the TV of Lambda around an evaluation of a predicate.
;; ----------------------------------------------------------------------

(define lambda-predicate-evaluation-vardecl
  (VariableList
     (TypedVariable
        (Variable "$V")
        (TypeChoice
           (Type "TypedVariableLink")
           (Type "VariableList")
           (Type "VariableNode")))
     (Variable "$P")
     (Variable "$Args")))

(define lambda-predicate-evaluation-pattern
  (And
    (Quote (Lambda
      (Unquote (VariableNode "$V"))
      (Unquote ((EvaluationLink
        (Variable "$P")
        (Variable "$Args"))))))
    (Evaluation
      (GroundedPredicate "scm: gt-zero-confidence")
      (Variable "$P"))))

(define lambda-predicate-evaluation-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: lambda-predicate-evaluation-formula")
     (ListLink
       (Quote (Lambda
         (Unquote (VariableNode "$V"))
         (Unquote ((EvaluationLink
           (Variable "$P")
           (Variable "$Args"))))))
       (Variable "$P"))))

(define lambda-predicate-evaluation-rule
  (BindLink
     lambda-predicate-evaluation-vardecl
     lambda-predicate-evaluation-pattern
     lambda-predicate-evaluation-rewrite))

(define (lambda-predicate-evaluation-formula lamb pred)
  (let ((pred-tv (cog-tv pred)))
    (if (tv-non-null-conf? pred-tv) ; Try to avoid constructing
                                    ; informationless knowledge
        (cog-merge-hi-conf-tv! lamb pred-tv))))

;; Name the rule
(define lambda-predicate-evaluation-rule-name
  (DefinedSchemaNode "lambda-predicate-evaluation-rule"))
(DefineLink lambda-predicate-evaluation-rule-name
  lambda-predicate-evaluation-rule)
