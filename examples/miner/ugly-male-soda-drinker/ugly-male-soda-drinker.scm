;; Load the miner module
(use-modules (opencog)
             (opencog miner))

;; ;; For debugging
;; (use-modules (opencog logger))
;; (cog-logger-set-level! "debug")
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-timestamp! #f)
;; (use-modules (opencog ure))
;; (ure-logger-set-level! "debug")
;; (ure-logger-set-sync! #t)
;; (ure-logger-set-timestamp! #f)

;; Load KB
(load "kb.scm")

;; Call the miner on the entire atomspace with minimum support of 5,
;; 100 iterations of forward chaining, incremental conjunction
;; expansion heuristic enabled, for expanding conjunctions up at most
;; 3 conjuncts.
;;
;; Expect to learn, among others, the following pattern
;;
;; (Lambda
;;   (Variable "$X")
;;   (Present
;;     (Inheritance
;;       (Variable "$X")
;;       (Concept "man"))
;;     (Inheritance
;;       (Variable "$X")
;;       (Concept "ugly"))
;;     (Inheritance
;;       (Variable "$X")
;;       (Concept "soda drinker"))))
;;
;; Because surprisingness is used the output is gonna be a sorted List
;; where the following
;;
;; (EvaluationLink (stv 0.88636364 1)
;;    (PredicateNode "isurp")
;;    (ListLink
;;       (LambdaLink
;;          (VariableNode "$PM-294b9483")
;;          (Present
;;             (InheritanceLink
;;                (VariableNode "$PM-294b9483")
;;                (ConceptNode "man")
;;             )
;;             (InheritanceLink
;;                (VariableNode "$PM-294b9483")
;;                (ConceptNode "ugly")
;;             )
;;             (InheritanceLink
;;                (VariableNode "$PM-294b9483")
;;                (ConceptNode "soda drinker")
;;             )
;;          )
;;       )
;;       (ConceptNode "texts-471337883-0XMuFQrSAV2O3H48")
;;    )
;; )
;;
;; appears on top as it is the most surprising pattern according to
;; the nisurp (I-Surprisingness) measure.
(define results (cog-mine (cog-atomspace)
                          #:minsup 5
                          #:maximum-iterations 100
                          #:incremental-expansion #t
                          #:max-conjuncts 3
                          #:max-variables 2
                          #:surprisingness 'nisurp))
