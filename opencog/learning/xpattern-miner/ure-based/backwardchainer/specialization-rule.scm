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

(cog-logger-set-level! "fine")
(cog-logger-set-stdout! #t)
(cog-logger-set-sync! #t)

;; For now we implement a simplified unary version of that rule
;;
;; Evaluation <tv1>
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       <x>
;;       <g>
;;     <ms>
;; <f-lamb>
;; |-
;; Evaluation <tv2>
;;   Predicate "minsup"
;;   List
;;     ComposeLink
;;       Lambda
;;         <x>
;;         <g>
;;       <f-lamb>
;;     <ms>
(define unary-specialization-rule
  (let* (;; Variables
         (x (Variable "$x"))
         (g (Variable "$g"))
         (ms (Variable "$ms"))
         (f-lamb (Variable "$f-lamb"))
         ;; Constants
         (minsup (Predicate "minsup"))
         ;; Types
         (VariableT (Type "VariableNode"))
         (NumberT (Type "NumberNode"))
         (LambdaT (Type "LambdaLink"))
         ;; Vardecls
         (x-decl (TypedVariable x VariableT))
         (g-decl g)
         (ms-decl (TypedVariable ms NumberT))
         (f-lamb-decl (TypedVariable f-lamb LambdaT))
         (vardecl (VariableList x-decl g-decl ms-decl f-lamb-decl))
         ;; Patterns
         (g-lamb (Quote (Lambda (Unquote x) (Unquote g))))
         (pattern (Evaluation
                    minsup
                    (List
                      g-lamb
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
                           (Quote (Compose
                             (Unquote g-lamb)
                             (Unquote f-lamb)))
                           ms))
                       pattern
                       f-lamb))))
    (Bind
      vardecl
      (And pattern f-lamb pre-condition)
      rewrite)))

(define (absolutely-true A)
  (bool->tv (tv->bool (cog-tv A))))

(define (unary-specialization-formula conclusion . premises)
  (cog-logger-debug "unary-specialization-formula conclusion = ~a, premises = ~a"
                    conclusion premises)
  (if (= (length premises) 2)
      (let* ((minsup-pred (car premises))
             (dummy-1 (cog-logger-debug "unary-specialization-formula minsup-pred = ~a" minsup-pred))
             (minsup-pred-tv (cog-tv minsup-pred))
             (dummy-2 (cog-logger-debug "unary-specialization-formula minsup-pred-tv = ~a" minsup-pred-tv))
             (f-lamb (cdr premises))
             (dummy-3 (cog-logger-debug "unary-specialization-formula f-lamb = ~a" f-lamb))
             (gf (gadr conclusion))
             (dummy-4 (cog-logger-debug "unary-specialization-formula gf = ~a" gf))
             (ms (atom->number (gddr conclusion)))
             (dummy-5 (cog-logger-debug "unary-specialization-formula ms = ~a" ms))
             (conclusion-tv (if (tv->bool minsup-pred-tv)
                                ;; g has enough support, let see if
                                ;; g.f has enough support
                                (bool->tv (support gf ms))
                                ;; g does not have enough support,
                                ;; therefore g.f doesn't have enough
                                ;; support
                                (stv 0 1)))
             (dummy-6 (cog-logger-debug "unary-specialization-formula conclusion-tv = ~a" conclusion-tv)))
        (cog-set-tv! conclusion conclusion-tv))))

;; TODO: move this to rule-engine utils
(define (atom->number A)
  (cog-logger-debug "atom->number A = ~a" A)
  (string->number (cog-name A)))

;; Return #t if L has a frequency equal to or greater than ms, #f otherwise
(define (support L ms)
  ;; TODO: clobber this with log messages
  (cog-logger-debug "support L = ~a, ms" L ms)
  (let* ((L-exec (cog-execute! L)))  ; consume compositions
    (if (= (cog-arity L) 2)
      (let* ((vardecl (gar L-exec))
             (body (gdr L-exec))
             (bl (Bind vardecl body body)) ; to deal with unordered links
             (results (cog-bind-first-n bl ms)))
        (= (cog-arity results) ms))
      ;; Supposedly no variable declaration
      (let* ((body (gar L-exec))
             (bl (Bind body body)) ; to deal with unordered links
             (results (cog-bind-first-n bl ms)))
        (= (cog-arity results) ms)))))
