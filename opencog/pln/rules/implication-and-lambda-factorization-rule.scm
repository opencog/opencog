;; =======================================================================
;; ImplicationLink AndLink Lambda Factorization Rule
;;
;; TODO: Replace this by higher order fact
;;
;; AndLink
;;    LambdaLink
;;       V
;;       A1
;;    ...
;;    LambdaLink
;;       V
;;       An
;; |-
;; LambdaLink
;;    V
;;    AndLink
;;       A1
;;       ...
;;       An
;;
;; where V is a variable or a list of variables, A1 to An are bodies
;; using containing variable(s) V.
;;
;; Also, the consequent will actually be
;;
;; ImplicationLink <1 1>
;;    AndLink
;;       LambdaLink
;;          V
;;          A1
;;       ...
;;       LambdaLink
;;          V
;;          An
;;    LambdaLink
;;       V
;;       AndLink
;;          A1
;;          ...
;;          An
;;
;; Because it is much easier to chain later on. This will be replaced
;; by higher order facts later.
;; -----------------------------------------------------------------------

(define implication-and-lambda-factorization-vardecl
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableNode")
           (TypeNode "VariableList")))
     ;; We intensionally restrict $A1 and $A2 to be EvaluationLink to
     ;; ensure that we don't produce indefinitely nested AndLinks
     (TypedVariableLink
        (VariableNode "$A1")
        (TypeNode "EvaluationLink"))
     (TypedVariableLink
        (VariableNode "$A2")
        (TypeNode "EvaluationLink"))))

(define implication-and-lambda-factorization-pattern
  (LocalQuoteLink                   ; Necessary so the AndLink doesn't
                                    ; count as a connective
     (AndLink
        (QuoteLink (LambdaLink
           (UnquoteLink (VariableNode "$TyVs"))
           (UnquoteLink (VariableNode "$A1"))))
        (QuoteLink (LambdaLink
           (UnquoteLink (VariableNode "$TyVs"))
           (UnquoteLink (VariableNode "$A2")))))))

(define implication-and-lambda-factorization-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-and-lambda-factorization-formula")
     (ImplicationLink
        (AndLink
           (QuoteLink (LambdaLink
              (UnquoteLink (VariableNode "$TyVs"))
              (UnquoteLink (VariableNode "$A1"))))
           (QuoteLink (LambdaLink
              (UnquoteLink (VariableNode "$TyVs"))
              (UnquoteLink (VariableNode "$A2")))))
        (QuoteLink (LambdaLink
           (UnquoteLink (VariableNode "$TyVs"))
           (UnquoteLink (AndLink
              (VariableNode "$A1")
              (VariableNode "$A2"))))))))

(define implication-and-lambda-factorization-rule
  (BindLink
     implication-and-lambda-factorization-vardecl
     implication-and-lambda-factorization-pattern
     implication-and-lambda-factorization-rewrite))

(define (implication-and-lambda-factorization-formula Impl)
  (cog-set-tv! Impl (stv 1 1)))

;; Name the rule
(define implication-and-lambda-factorization-rule-name
  (DefinedSchemaNode "implication-and-lambda-factorization-rule"))
(DefineLink implication-and-lambda-factorization-rule-name
  implication-and-lambda-factorization-rule)
