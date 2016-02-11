;; =======================================================================
;; AndLink Lambda Distribution Rule
;; (TODO add wiki page)
;;
;; LambdaLink
;;    V
;;    AndLink
;;       A1
;;       ...
;;       An
;; |-
;; AndLink
;;    LambdaLink
;;       V
;;       A1
;;    ...
;;    LambdaLink
;;       V
;;       An
;;
;; where V is a variable or a list of variables, A1 to An are bodies
;; using containing variable(s) V.
;; -----------------------------------------------------------------------

(define and-lambda-distribution-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableNode")
           (TypeNode "VariableList")))
     (TypedVariableLink
        (VariableNode "$And")
        (TypeNode "AndLink"))))

(define and-lambda-distribution-body
  (LambdaLink
     (VariableNode "$TyVs")
     (VariableNode "$And")))

(define and-lambda-distribution-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: and-lambda-distribution-formula")
     (ListLink
        and-lambda-distribution-body)))

(define and-lambda-distribution-rule
  (BindLink
     and-lambda-distribution-variables
     and-lambda-distribution-body
     and-lambda-distribution-rewrite))

(define (and-lambda-distribution-formula Lamb)
  (let* (
         (Lamb-outgoings (cog-outgoing-set Lamb))
         (SV (car Lamb-outgoings))
         (conjunction (cadr Lamb-outgoings))
         (junctors (cog-outgoing-set conjunction))
         (wrap_with_lambda (lambda (junctor) (LambdaLink SV junctor)))
         (wrapped_junctors (map wrap_with_lambda junctors)))
    (cog-set-tv! (apply AndLink wrapped_junctors) (cog-tv Lamb))))

;; Name the rule
(define and-lambda-distribution-rule-name
  (DefinedSchemaNode "and-lambda-distribution-rule"))
(DefineLink and-lambda-distribution-rule-name
  and-lambda-distribution-rule)
