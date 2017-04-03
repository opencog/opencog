;; =======================================================================
;; AndLink Lambda Factorization Rule
;;
;; WARNING: Not BC compatible.
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
;; Also, the consequent will actually be the doudble implication
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
;; ImplicationLink <1 1>
;;    LambdaLink
;;       V
;;       AndLink
;;          A1
;;          ...
;;          An
;;    AndLink
;;       LambdaLink
;;          V
;;          A1
;;       ...
;;       LambdaLink
;;          V
;;          An
;;
;; Because it is much easier to chain later on. This will be replaced
;; by higher order facts later.
;; -----------------------------------------------------------------------

(define and-lambda-factorization-double-implication-variables
  (VariableList
    (TypedVariableLink
      (VariableNode "$TyVs")
      (TypeChoice
        (TypeNode "TypedVariableLink")
        (TypeNode "VariableNode")
        (TypeNode "VariableList")))
    (VariableNode "$A1")
    (VariableNode "$A2")))

(define and-lambda-factorization-double-implication-body
  (LocalQuoteLink                   ; Necessary so the AndLink doesn't
                                    ; count as a connective
    (AndLink
      (QuoteLink (LambdaLink
        (VariableNode "$TyVs")
        (VariableNode "$A1")))
      (QuoteLink (LambdaLink
        (VariableNode "$TyVs")
        (VariableNode "$A2"))))))

(define and-lambda-factorization-double-implication-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: and-lambda-factorization-double-implication-formula")
     (ListLink
        (VariableNode "$TyVs")
        (VariableNode "$A1")
        (VariableNode "$A2"))))

(define and-lambda-factorization-double-implication-rule
  (BindLink
     and-lambda-factorization-double-implication-variables
     and-lambda-factorization-double-implication-body
     and-lambda-factorization-double-implication-rewrite))

(define (and-lambda-factorization-double-implication-formula var a1 a2)
  (let* ((and-lamb (AndLink (LambdaLink var a1) (LambdaLink var a2)))
        (and-lamb-s (cog-stv-strength and-lamb))
        (and-lamb-c (cog-stv-confidence and-lamb))
        (lamb (LambdaLink var (cog-new-flattened-link 'AndLink a1 a2))))
    (if (and (< 1e-8 and-lamb-s) (< 1e-8 and-lamb-c))
        (let ((lamb-s and-lamb-s)
              (lamb-c and-lamb-c))
          (cog-merge-hi-conf-tv! lamb (cog-tv and-lamb))
          (List
           (cog-set-tv! (ImplicationLink and-lamb lamb) (stv 1 1))
           (cog-set-tv! (ImplicationLink lamb and-lamb) (stv 1 1))))
        (cog-undefined-handle))))

;; Name the rule
(define and-lambda-factorization-double-implication-rule-name
  (DefinedSchemaNode "and-lambda-factorization-double-implication-rule"))
(DefineLink and-lambda-factorization-double-implication-rule-name
  and-lambda-factorization-double-implication-rule)
