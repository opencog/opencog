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

(define implication-and-lambda-factorization-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableNode")
           (TypeNode "VariableList")))
     (VariableNode "$A1")
     (VariableNode "$A2")))

(define implication-and-lambda-factorization-body
  (QuoteLink                        ; Necessary so the AndLink doesn't
                                    ; count as a connective
     (AndLink
        (UnquoteLink
           (LambdaLink
              (VariableNode "$TyVs")
              (VariableNode "$A1")))
        (UnquoteLink
           (LambdaLink
              (VariableNode "$TyVs")
              (VariableNode "$A2"))))))

(define implication-and-lambda-factorization-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-and-lambda-factorization-formula")
     (ListLink
        (VariableNode "$TyVs")
        (VariableNode "$A1")
        (VariableNode "$A2"))))

(define implication-and-lambda-factorization-rule
  (BindLink
     implication-and-lambda-factorization-variables
     implication-and-lambda-factorization-body
     implication-and-lambda-factorization-rewrite))

(define (implication-and-lambda-factorization-formula var a1 a2)
  (let ((and-lamb (AndLink (LambdaLink var a1) (LambdaLink var a2)))
        (lamb (LambdaLink var (cog-new-flattened-link 'AndLink a1 a2))))
    (cog-set-tv! lamb (cog-tv and-lamb))
    (cog-set-tv! (ImplicationLink and-lamb lamb) (stv 1 1))))

;; Name the rule
(define implication-and-lambda-factorization-rule-name
  (DefinedSchemaNode "implication-and-lambda-factorization-rule"))
(DefineLink implication-and-lambda-factorization-rule-name
  implication-and-lambda-factorization-rule)
