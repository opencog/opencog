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
;;
;; * mv is the maximum of variables allowed in the resulting patterns.
(define (gen-conjunction-expansion-rule nary mv)
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
            (Present
              minsup-f
              minsup-g)
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
            (GroundedSchema (string-append "scm: conjunction-expansion-mv-"
                                           (number->string mv)
                                           "-formula"))
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
          (GroundedSchema (string-append "scm: conjunction-expansion-mv-"
                                         (number->string mv)
                                         "-formula"))
          (List
            ;; Fake conclusion, since we can't statistically define its
            ;; pattern ATM
            (minsup-eval (top) texts ms)
            ;; Premises, wrap in Set because their order does not matter
            (Set minsup-f
                 minsup-g)))))))

;; Conjunction expansion formula
(define (gen-conjunction-expansion-formula mv)
  (lambda (conclusion . premises)
    ;; (cog-logger-debug "conjunction-expansion-formula mv = ~a, conclusion = ~a, premises = ~a" mv conclusion premises)
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
                        (cog-expand-conjunction f g texts ms (Number mv))
                        (cog-expand-conjunction g f texts ms (Number mv))))
               (mk-minsup (lambda (fg) (minsup-eval-true fg texts ms)))
               ;; cog-expand-conjunction only return patterns with
               ;; enough support
               (minsup-fgs (map mk-minsup (cog-outgoing-set fgs))))
          (Set minsup-fgs)))))

;; Instantiate conjunction expansion formulae for different maximum
;; number of variables
(define conjunction-expansion-mv-1-formula (gen-conjunction-expansion-formula 1))
(define conjunction-expansion-mv-2-formula (gen-conjunction-expansion-formula 2))
(define conjunction-expansion-mv-3-formula (gen-conjunction-expansion-formula 3))
(define conjunction-expansion-mv-4-formula (gen-conjunction-expansion-formula 4))
(define conjunction-expansion-mv-5-formula (gen-conjunction-expansion-formula 5))
(define conjunction-expansion-mv-6-formula (gen-conjunction-expansion-formula 6))
(define conjunction-expansion-mv-7-formula (gen-conjunction-expansion-formula 7))
(define conjunction-expansion-mv-8-formula (gen-conjunction-expansion-formula 8))
(define conjunction-expansion-mv-9-formula (gen-conjunction-expansion-formula 9))
