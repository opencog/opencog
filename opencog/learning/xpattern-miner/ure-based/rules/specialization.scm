;; Rule to specialize a pattern by composing it with a shallow
;; abstraction, a constant, or a variable, and checks that it has
;; enough support.
;;
;; Given g with arity n and with support ms, and f, specialize g by
;; composing it with f over one of its variables, xi.
;;
;; Evaluation <tv1>
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       VariableList
;;         <x1>
;;         ...
;;         <xn>
;;       <g-body>
;;     <ms>
;; <f>
;; |-
;; Evaluation <tv2>
;;   Predicate "minsup"
;;   List
;;     Put
;;       Lambda
;;         VariableList
;;           <x1>
;;           ...
;;           <xn>
;;         <g-body>
;;       List
;;         <x1>
;;         ...
;;         <xi-1>
;;         <f>
;;         <xi+1>
;;         ...
;;         <xn>
;;     <ms>
;;
;; assuming that tv1 equals to (stv 1 1), then calculate the frequency
;; of the composed pattern and set tv2 accordingly, (stv 1 1) if g
;; composed with f has support ms, (stv 0 1) otherwise.
;;
;; <f> may either a shallow pattern (a function), a constant or a
;; variable amongst <x1> to <xn> different than <xi>.
;;
;; TODO: we might want to split such rule into 2,
;;
;; 1. Relating PutLink and specialization/abstraction
;;
;; 2. Relating specialization/abstraction and minsup
;;
;; instead of being mangled into one rule.

(load "pattern-miner-utils.scm")

;; Generate composition specialization rule for a given arity and
;; argument index ranging from 0 to arity-1.
;;
;; For instance (gen-specialization-rule 2 1) generates the following rule
;;
;; Evaluation <stv 1 1>
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       VariableList
;;         <x0>
;;         <x1>
;;       <g-body>
;;     <texts>
;;     <ms>
;; <f>
;; |-
;; Evaluation <tv>
;;   Predicate "minsup"
;;   List
;;     Put
;;       Lambda
;;         VariableList
;;           <x0>
;;           <x1>
;;         <g-body>
;;       List
;;         <x0>
;;         <f>
;;     <texts>
;;     <ms>
(define (gen-specialization-rule arity index)
  (let* (;; Variables
         (xs (gen-variables "$spe-arg" arity))
         (g (Variable "$g"))
         (texts (Variable "$texts"))
         (ms (Variable "$ms"))
         (f (Variable "$f"))
         ;; Types
         (NumberT (Type "NumberNode"))
         (LambdaT (Type "LambdaLink"))
         (ConceptT (Type "ConceptNode"))
         (VariableT (Type "VariableNode"))
         (PutT (Type "PutLink"))
         ;; Vardecls
         (g-decl (TypedVariable g (TypeChoice LambdaT PutT)))
         (texts-decl (TypedVariable texts ConceptT))
         (ms-decl (TypedVariable ms NumberT))
         ;; TODO: for now hardwire the types of f, then later only
         ;; accept shallow abstractions, constants from valuations, or
         ;; variables from the variables of g.
         (f-decl (TypedVariable f (TypeChoice LambdaT ConceptT VariableT)))
         (vardecl (VariableList g-decl texts-decl ms-decl f-decl))
         ;; Patterns
         (minsup-g (minsup g texts ms))
         ;; Make sure the pattern has the minimum support
         (pre-cond-1 (Evaluation
                       (GroundedPredicate "scm: absolutely-true")
                       minsup-g))
         (pre-cond-2 (Evaluation
                       (GroundedPredicate "scm: has-arity")
                       (List
                         g
                         (Number arity))))
         ;; Rewrite
         (xs-f (if (< 1 (length xs)) (List (replace-el xs index f)) f))
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: specialization-formula")
                    (List
                      (minsup
                        (Quote (Put
                          (Unquote g)
                          (Unquote xs-f)))
                        texts
                        ms)
                      minsup-g
                      f))))
    (Bind
      vardecl
      (And minsup-g f pre-cond-1 pre-cond-2)
      rewrite)))

(define (specialization-formula conclusion . premises)
  ;; (cog-logger-debug "specialization-formula conclusion = ~a, premises = ~a"
  ;;                   conclusion premises)
  (if (= (length premises) 2)
      (let* ((pre-minsup-pred (car premises))
             (con-minsup-args (gdr conclusion))
             (pre-minsup-pred-tv (cog-tv pre-minsup-pred))
             (f-lamb (car (cdr premises)))
             (gf (cog-outgoing-atom con-minsup-args 0))
             (texts (cog-outgoing-atom con-minsup-args 1))
             (ms-atom (cog-outgoing-atom con-minsup-args 2))
             (ms (inexact->exact (atom->number ms-atom)))
             (conclusion-tv (if (tv->bool pre-minsup-pred-tv)
                                ;; g has enough support, let see if
                                ;; g.f has enough support
                                (let ((sup (support gf texts ms)))
                                  (if (<= ms sup)
                                      (stv 1 1)
                                      #f)) ; It is ill-formed
                                ;; g does not have enough support,
                                ;; therefore g.f doesn't have enough
                                ;; support
                                #f))
             ;; Reduce the pattern to a normal form by collapsing all
             ;; PutLinks, and output this norma form
             (reduced-conclusion (cog-execute! conclusion)))
        (if conclusion-tv
            (cog-set-tv! reduced-conclusion conclusion-tv)))))

;; Define unary specialization
(define unary-specialization-rule
  (gen-specialization-rule 1 0))
(define unary-specialization-rule-name
  (DefinedSchemaNode "unary-specialization-rule"))
(DefineLink unary-specialization-rule-name
  unary-specialization-rule)

;; Define binary specialization
(define binary-first-arg-specialization-rule
  (gen-specialization-rule 2 0))
(define binary-first-arg-specialization-rule-name
  (DefinedSchemaNode "binary-first-arg-specialization-rule"))
(DefineLink binary-first-arg-specialization-rule-name
  binary-first-arg-specialization-rule)

(define binary-second-arg-specialization-rule
  (gen-specialization-rule 2 1))
(define binary-second-arg-specialization-rule-name
  (DefinedSchemaNode "binary-second-arg-specialization-rule"))
(DefineLink binary-second-arg-specialization-rule-name
  binary-second-arg-specialization-rule)

;; Define ternary specialization
(define ternary-first-arg-specialization-rule
  (gen-specialization-rule 3 0))
(define ternary-first-arg-specialization-rule-name
  (DefinedSchemaNode "ternary-first-arg-specialization-rule"))
(DefineLink ternary-first-arg-specialization-rule-name
  ternary-first-arg-specialization-rule)

(define ternary-second-arg-specialization-rule
  (gen-specialization-rule 3 1))
(define ternary-second-arg-specialization-rule-name
  (DefinedSchemaNode "ternary-second-arg-specialization-rule"))
(DefineLink ternary-second-arg-specialization-rule-name
  ternary-second-arg-specialization-rule)

(define ternary-third-arg-specialization-rule
  (gen-specialization-rule 3 1))
(define ternary-third-arg-specialization-rule-name
  (DefinedSchemaNode "ternary-third-arg-specialization-rule"))
(DefineLink ternary-third-arg-specialization-rule-name
  ternary-third-arg-specialization-rule)
