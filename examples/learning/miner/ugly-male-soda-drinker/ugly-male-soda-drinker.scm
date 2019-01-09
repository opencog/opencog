;; Load the miner module
(use-modules (opencog)
             (opencog miner))

;; ;; For debugging
;; (use-modules (opencog logger))
;; (use-modules (opencog rule-engine))
;; (ure-logger-set-level! "debug")

;; Load KB
(load "kb.scm")

;; Call the miner on the entire atomspace with minimum support of 5,
;; 100 iterations of forward chaining, a 0.1 strength and confidence
;; of expanding conjunctions up at most 3 conjuncts.
;;
;; Expect to learn, among others, the following pattern
;;
;; (Lambda
;;   (Variable "$X")
;;   (And
;;     (Inheritance
;;       (Variable "$X")
;;       (Concept "man"))
;;     (Inheritance
;;       (Variable "$X")
;;       (Concept "ugly"))
;;     (Inheritance
;;       (Variable "$X")
;;       (Concept "soda drinker"))))
(define results (cog-mine (cog-atomspace)
                          #:minsup 5
                          #:maxiter 100
                          #:incremental-expansion #t
                          #:max-conjuncts 3))
