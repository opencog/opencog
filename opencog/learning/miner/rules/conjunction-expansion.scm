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

;; Generate a conjunction expansion rule so that the left pattern, f,
;; is a nary conjunction. Lets call f the left conjunction. Thus the
;; produced pattern, will be a nary+1 conjunction, and the right
;; pattern, g, is a unary conjunction.
;;
;; * nary <= 0: generate a rule that doesn't restrict the arity of the
;;              left conjunction.
;;
;; * 0 < nary: generate a rule that restricts the arity of the left
;;             conjunction to nary.
(define (gen-conjunction-expansion-rule nary)
  ;; Shared variables
  (define f-vardecl (Variable "$f-vardecl"))
  (define g-vardecl (Variable "$g-vardecl"))
  (define texts (Variable "$texts"))
  (define ms (Variable "$ms"))
  (define g-body (Variable "$g-body"))
  ;; Shared types
  (define ConceptT (Type "ConceptNode"))
  (define NumberT (Type "NumberNode"))
  (define AndT (Type "AndLink"))
  ;; Shared variable declarations
  (define f-vardecl-decl f-vardecl)
  (define g-vardecl-decl g-vardecl)
  (define g-body-decl g-body)
  (define texts-decl (TypedVariable texts ConceptT))
  (define ms-decl (TypedVariable ms NumberT))
  ;; Shared clauses
  (define g (Quote (Lambda (Unquote g-vardecl) (Unquote g-body))))
  (define minsup-g (minsup-eval g texts ms))

  ;; Generate rule by nary cases
  (if (<= nary 1)
      ;; There are 2 cases
      ;; 1. no restriction on the arity of the left conjunction,
      ;; 2. unary left conjunction
      (let* (;; Variables
             (f-body (Variable "$f-body"))
             ;; Vardecls
             (f-body-decl f-body)
             ;; clauses
             (f (Quote (Lambda (Unquote f-vardecl) (Unquote f-body))))
             (minsup-f (minsup-eval f texts ms)))
        (Bind
          (VariableList
            f-vardecl-decl
            g-vardecl-decl
            f-body-decl
            g-body-decl
            texts-decl
            ms-decl)
          (And
            minsup-f
            minsup-g
            (absolutely-true-eval minsup-f)
            (absolutely-true-eval minsup-g)
            (not-equal-top f)
            (not-equal-top g)
            (unary-conjunction-eval g-body)
            ;; If nary equals to 1, make sure the left conjunction is unary
            (if (= nary 1)
                (unary-conjunction-eval f-body)
                '()))
          (ExecutionOutput
            (GroundedSchema "scm: conjunction-expansion-formula")
            (List
              ;; Fake conclusion, since we can't statistically define its
              ;; pattern ATM
              (minsup-eval (top) texts ms)
              ;; Premises, wrap in Set because their order does not matter
              (Set minsup-f
                   minsup-g)))))

      ;; Left pattern has 1 < nary arity
      (let* (;; Variables
             (f-conjuncts (gen-variables "$f-conjunct" nary))
             ;; Vardecls
             (f-conjuncts-decls f-conjuncts)
             ;; clauses
             (f (Quote (Lambda (Unquote f-vardecl) (And (map Unquote f-conjuncts)))))
             (minsup-f (minsup-eval f texts ms)))
      (Bind
        (VariableList
          f-vardecl-decl
          g-vardecl-decl
          f-conjuncts-decls
          g-body-decl
          texts-decl
          ms-decl)
        (And
          minsup-f
          minsup-g
          (absolutely-true-eval minsup-f)
          (absolutely-true-eval minsup-g)
          (not-equal-top g)
          (unary-conjunction-eval g-body))
        (ExecutionOutput
          (GroundedSchema "scm: conjunction-expansion-formula")
          (List
            ;; Fake conclusion, since we can't statistically define its
            ;; pattern ATM
            (minsup-eval (top) texts ms)
            ;; Premises, wrap in Set because their order does not matter
            (Set minsup-f
                 minsup-g)))))))

;; Conjunction expansion formula
(define (conjunction-expansion-formula conclusion . premises)
  ;; (cog-logger-debug "conjunction-expansion-formula conclusion = ~a, premises = ~a" conclusion premises)
  (if (= (length premises) 1)
      (let* ((minsup-fg (car premises))
             (minsup-f (cog-outgoing-atom minsup-fg 0))
             (minsup-g (cog-outgoing-atom minsup-fg 1))
             (f (get-pattern minsup-f))
             (g (get-pattern minsup-g))
             (texts (get-texts minsup-f))
             (ms (get-ms minsup-f))
             ;; Swap f and g to make sure the second argument of
             ;; cog-expand-conjunction is never a conjunction
             (fgs (if (unary-conjunction? (get-body g))
                      (cog-expand-conjunction f g texts ms)
                      (cog-expand-conjunction g f texts ms)))
             (mk-minsup (lambda (fg) (minsup-eval-true fg texts ms)))
             ;; cog-expand-conjunction only return patterns with
             ;; enough support
             (minsup-fgs (map mk-minsup (cog-outgoing-set fgs))))
        (Set minsup-fgs))))

;; Define arbitrary nary conjunction expansion
(define conjunction-expansion-rule-name
  (DefinedSchemaNode "conjunction-expansion-rule"))
(DefineLink conjunction-expansion-rule-name
  (gen-conjunction-expansion-rule 0))

;; Define unary conjunction expansion
(define conjunction-expansion-1ary-rule-name
  (DefinedSchemaNode "conjunction-expansion-1ary-rule"))
(DefineLink conjunction-expansion-1ary-rule-name
  (gen-conjunction-expansion-rule 1))

;; Define binary conjunction expansion
(define conjunction-expansion-2ary-rule-name
  (DefinedSchemaNode "conjunction-expansion-2ary-rule"))
(DefineLink conjunction-expansion-2ary-rule-name
  (gen-conjunction-expansion-rule 2))

;; Define ternary conjunction expansion
(define conjunction-expansion-3ary-rule-name
  (DefinedSchemaNode "conjunction-expansion-3ary-rule"))
(DefineLink conjunction-expansion-3ary-rule-name
  (gen-conjunction-expansion-rule 3))

;; Define quaternary conjunction expansion
(define conjunction-expansion-4ary-rule-name
  (DefinedSchemaNode "conjunction-expansion-4ary-rule"))
(DefineLink conjunction-expansion-4ary-rule-name
  (gen-conjunction-expansion-rule 4))

;; Define quaternary conjunction expansion
(define conjunction-expansion-5ary-rule-name
  (DefinedSchemaNode "conjunction-expansion-5ary-rule"))
(DefineLink conjunction-expansion-5ary-rule-name
  (gen-conjunction-expansion-rule 5))
