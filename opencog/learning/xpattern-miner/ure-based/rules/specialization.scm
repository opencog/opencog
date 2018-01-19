;; TODO:
;; 1. Update comment to use PutLink and texts
;; 2. Also we probably want to split rules about
;;    specialization/abstraction and minsup.
;;
;; Given n+1 patterns
;;
;; - an n-ary pattern, g with frequency equal to or above ms
;;
;; - n m-ary patterns f1 to fn
;;
;; specialize g to produce a pattern g(f1(x1,...,xm),...,fn(x1,...,xm))
;;
;; Evaluation <tv1>
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       VariableList
;;         <x1>
;;         ...
;;         <xn>
;;       <g>
;;     <ms>
;; Lambda
;;   VariableList
;;     <x1>
;;     ...
;;     <xm>
;;   <f1>
;; ...
;; Lambda
;;   VariableList
;;     <x1>
;;     ...
;;     <xm>
;;   <f1>
;; |-
;; Evaluation <tv2>
;;   Predicate "minsup"
;;   List
;;     ComposeLink
;;       Lambda
;;         VariableList
;;           <x1>
;;           ...
;;           <xn>
;;         <g>
;;       List
;;         Lambda
;;           VariableList
;;             <x1>
;;             ...
;;             <xm>
;;           <f1>
;;         ...
;;         Lambda
;;           VariableList
;;             <x1>
;;             ...
;;             <xm>
;;           <f1>
;;     <ms>
;;
;; assuming that tv1 equals to (stv 1 1), then calculate the frequency
;; of the composed pattern and set tv2 accordingly.

(use-modules (opencog logger))
(use-modules (opencog query))
(use-modules (opencog rule-engine))

(load "pattern-miner-utils.scm")

;; (cog-logger-set-level! "fine")
;; (cog-logger-set-stdout! #t)
;; (cog-logger-set-sync! #t)

;; For now we implement a simplified unary version of that rule
;;
;; Evaluation <tv1>
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       <x>
;;       <g>
;;     <texts>
;;     <ms>
;; <f-lamb>
;; |-
;; Evaluation <tv2>
;;   Predicate "minsup"
;;   List
;;     PutLink
;;       Lambda
;;         <x>
;;         <g>
;;       <f-lamb>
;;     <texts>
;;     <ms>
(define unary-specialization-rule
  (let* (;; Variables
         (x (Variable "$x"))
         (g (Variable "$g"))
         (texts (Variable "$texts"))
         (ms (Variable "$ms"))
         (f-lamb (Variable "$f-lamb"))
         ;; Constants
         (minsup (Predicate "minsup"))
         ;; Types
         (VariableT (Type "VariableNode"))
         (NumberT (Type "NumberNode"))
         (LambdaT (Type "LambdaLink"))
         (ConceptT (Type "ConceptNode"))
         ;; Vardecls
         (x-decl (TypedVariable x VariableT))
         (g-decl g)
         (texts-decl (TypedVariable texts ConceptT))
         (ms-decl (TypedVariable ms NumberT))
         (f-lamb-decl (TypedVariable f-lamb LambdaT))
         (vardecl (VariableList x-decl g-decl texts-decl
                                ms-decl f-lamb-decl))
         ;; Patterns
         (g-lamb (Quote (Lambda (Unquote x) (Unquote g))))
         (pattern (Evaluation
                    minsup
                    (List
                      g-lamb
                      texts
                      ms)))
         ;; Make sure the pattern has the minimum support
         (pre-condition (Evaluation
                          (GroundedPredicate "scm: absolutely-true")
                          pattern))
         ;; Rewrite
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: unary-specialization-formula")
                    (List
                      (Evaluation
                        minsup
                        (List
                          (Quote (Put
                            (Unquote g-lamb)
                            (Unquote f-lamb)))
                          ms))
                      pattern
                      f-lamb))))
    (Bind
      vardecl
      (And pattern f-lamb pre-condition)
      rewrite)))

(define (unary-specialization-formula conclusion . premises)
  (if (= (length premises) 2)
      (let* ((minsup-pred (car premises))
             (minsup-pred-tv (cog-tv minsup-pred))
             (f-lamb (cdr premises))
             (gf (gadr conclusion))
             (ms (inexact->exact (atom->number (gddr conclusion))))
             (conclusion-tv (if (tv->bool minsup-pred-tv)
                                ;; g has enough support, let see if
                                ;; g.f has enough support
                                (let ((sup (support gf ms)))
                                  (if sup
                                      (bool->tv (= ms sup))
                                      #f)) ; It is ill-formed
                                ;; g does not have enough support,
                                ;; therefore g.f doesn't have enough
                                ;; support
                                (stv 0 1))))
        (if conclusion-tv
            (cog-set-tv! conclusion conclusion-tv)))))

(define unary-specialization-rule-name
  (DefinedSchemaNode "unary-specialization-rule"))
(DefineLink unary-specialization-rule-name
  unary-specialization-rule)
