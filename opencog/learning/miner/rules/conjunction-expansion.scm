;; Rule to expand a conjunction with an existing pattern given that
;; both the conjunction and the existing pattern have enough support.
;;
;; Semi-formally
;;
;; Evaluation (stv 1 1)
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       <f-vardecl>
;;       <f-body>
;;     <texts>
;;     <ms>
;; Evaluation (stv 1 1)
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       <g-vardecl>
;;       <g-body>
;;     <texts>
;;     <ms>
;; |-
;; Evaluation (stv 1 1)
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       VariableList
;;         <f-vardecl>
;;         <g-vardecl>
;;       And
;;         <f-body>
;;         <g-body>
;;     <texts>
;;     <ms>
;;
;; Where <f-body> may be a conjunction of multiple conjuncts, while
;; <g-body> may only be a single conjunct. Also it is assumed that
;; variables are properly alpha-converted to avoid any variable name
;; collision. Finally,
;;
;; VariableList
;;   <f-vardecl>
;;   <g-vardecl>
;;
;; and
;;
;; And
;;   <f-body>
;;   <g-body>
;;
;; are flattened.

(load "miner-rule-utils.scm")

(define conjunction-expansion-rule
  (let* (;; Variables
         (f-vardecl (Variable "$f-vardecl"))
         (g-vardecl (Variable "$g-vardecl"))
         (f-body (Variable "$f-body"))
         (g-body (Variable "$g-body"))
         (texts (Variable "$texts"))
         (ms (Variable "$ms"))
         ;; Types
         (ConceptT (Type "ConceptNode"))
         (NumberT (Type "NumberNode"))
         ;; Vardecls
         (texts-decl (TypedVariable texts ConceptT))
         (ms-decl (TypedVariable ms NumberT))
         ;; clauses
         (f (Quote (Lambda (Unquote f-vardecl) (Unquote f-body))))
         (g (Quote (Lambda (Unquote g-vardecl) (Unquote g-body))))
         (minsup-f (minsup-eval f texts ms))
         (minsup-g (minsup-eval g texts ms)))
    (Bind
      (VariableList
        f-vardecl
        g-vardecl
        f-body
        g-body
        texts-decl
        ms-decl)
      (And
        minsup-f
        minsup-g
        (absolutely-true-eval minsup-f)
        (absolutely-true-eval minsup-g)
        (single-conjunct-eval g-body)
        (Not (Equal f (top)))
        (Not (Equal g (top))))
      (ExecutionOutput
        (GroundedSchema "scm: conjunction-expansion-formula")
        (List
          ;; Fake conclusion, since we can't statistically define its
          ;; pattern ATM
          (minsup-eval (top) texts ms)
          ;; Premises
          minsup-f
          minsup-g)))))

;; Conjunction expansion formula
(define (conjunction-expansion-formula conclusion . premises)
  ;; (cog-logger-debug "conjunction-expansion-formula conclusion = ~a, premises = ~a" conclusion premises)
  (if (= (length premises) 2)
      (let* ((minsup-f (car premises))
             (minsup-g (cadr premises))
             (f (get-pattern minsup-f))
             (g (get-pattern minsup-g))
             (fg (cog-expand-conjunction f g))
             (texts (get-texts minsup-f))
             (ms (get-ms minsup-f)))
        (minsup-eval-true fg texts ms))))

;; Define conjunction expansion
(define conjunction-expansion-rule-name
  (DefinedSchemaNode "conjunction-expansion-rule"))
(DefineLink conjunction-expansion-rule-name
  conjunction-expansion-rule)
