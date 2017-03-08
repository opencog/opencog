;; =======================================================================
;; AndLink Lambda Distribution Rule
;;
;; LambdaLink
;;    <V>
;;    <Junctor>
;;       <A1>
;;       ...
;;       <An>
;; <=>
;; <Junctor>
;;    LambdaLink
;;       <V>
;;       <A1>
;;    ...
;;    LambdaLink
;;       <V>
;;       <An>
;;
;; where V is a variable or a list of variables, A1 to An are bodies
;; using containing variable(s) V and junctor is AndLink or OrLink.
;; -----------------------------------------------------------------------

;; Generate a junction-lambda-distribution-fact with arity N on the
;; junctor (AndLink or OrLink). For instance if junc is AndLink and N
;; is 2 it will generate
;;
;; EquivalenceScopeLink <1 1>
;;    VariableList
;;       TypedVariableLink
;;          VariableNode "$TyVs"
;;          TypeChoice
;;             TypeNode "TypedVariableLink"
;;             TypeNode "VariableNode"
;;             TypeNode "VariableList"
;;       VariableNode "$Body-0"
;;       VariableNode "$Body-1"
;;    QuoteLink LambdaLink
;;       UnquoteLink VariableNode "$TyVs"
;;       UnquoteLink AndLink
;;          VariableNode "$Body-0"
;;          VariableNode "$Body-1"
;;    AndLink
;;       QuoteLink LambdaLink
;;          UnquoteLink VariableNode "$TyVs"
;;          UnquoteLink VariableNode "$Body-0"
;;       QuoteLink LambdaLink
;;          UnquoteLink VariableNode "$TyVs"
;;          UnquoteLink VariableNode "$Body-1"
;;
;; TODO: add quotations
(define (gen-junc-lambda-distribution-fact junc N)
  (let* (
         ;; Build equivalence variables
         (TyVs (VariableNode "$TyVs"))
         (var-var (TypedVariableLink
                     TyVs
                     (TypeChoice
                        (TypeNode "TypedVariableLink")
                        (TypeNode "VariableNode")
                        (TypeNode "VariableList"))))
         (gen-conjuctee-name (lambda (i)
                               (string-append "$Body-"
                                              (number->string i))))
         (conjuctee-names (map gen-conjuctee-name (iota N)))
         (var-conjuctees (map VariableNode conjuctee-names))
         (var-decl-list (cons var-var var-conjuctees))
         (variables (VariableList var-decl-list))
         ;; Build left and right terms
         (gen-lambda (lambda (v) (LambdaLink TyVs v)))
         (lambdas (map gen-lambda var-conjuctees))
         (left-term (LambdaLink TyVs (apply junc var-conjuctees)))
         (right-term (apply junc lambdas)))
    (EquivalenceLink (stv 1 1)
       variables
       left-term
       right-term)))

;; Predefined likely most useful junctor lambda distribution facts

(define and-lambda-distribution-1-fact
  (gen-junc-lambda-distribution-fact AndLink 1))
(define and-lambda-distribution-2-fact
  (gen-junc-lambda-distribution-fact AndLink 2))
(define and-lambda-distribution-3-fact
  (gen-junc-lambda-distribution-fact AndLink 3))

(define or-lambda-distribution-1-fact
  (gen-junc-lambda-distribution-fact OrLink 1))
(define or-lambda-distribution-2-fact
  (gen-junc-lambda-distribution-fact OrLink 2))
(define or-lambda-distribution-3-fact
  (gen-junc-lambda-distribution-fact OrLink 3))
