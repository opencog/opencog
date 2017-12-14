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
;;     CompositionLink
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

(use-modules (opencog query))
(use-modules (opencog logger))

(cog-logger-set-level! "debug")
(cog-logger-set-stdout! #t)

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
;;     CompositionLink
;;       Lambda
;;         <x>
;;         <g>
;;       <f-lamb>
;;     <ms>
(define unary-specialization-rule
  (let* (;; Variables
         (x (Variable "$x"))
         (g (Variable "$g"))
         (f-lamb (Variable "$f-lamb"))
         ;; Constants
         (ms (Variable "$ms"))
         (minsup (Predicate "minsup"))
         ;; Types
         (VariableT (Type "VariableNode"))
         (LambdaT (Type "LambdaLink"))
         ;; Vardecls
         (x-decl (TypedVariable x VariableT))
         (g-decl g)
         (f-lamb-decl (TypedVariable f-lamb LambdaT))
         (vardecl (VariableList x-decl g-decl f-lamb-decl))
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
                           (Composition
                             g-lamb
                             f-lamb)
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
      (let* ((minsup-pred (gar premises))
             (minsup-pred-tv (cog-tv minsup-pred))
             (f-lamb (gdr premises))
             (gf (gadr conclusion))
             (ms (atom->number (gddr conclusion))))
        (if (tv->bool minsup-pred-tv)
            ;; g has enough support, let see if g.f has enough support
            (cog-tv! conclusion (bool->tv (support gf ms)))

            ;; g does not have enough support, therefore g.f doesn't
            ;; have enough support
            (cog-tv! conclusion (stv 0 1))))))

;; TODO: move this to rule-engine utils
(define (atom->number A)
  (string->number (cog-name A)))

;; Return #t if L has a frequency equal to or greater than ms, #f otherwise
(define (support L ms)
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
