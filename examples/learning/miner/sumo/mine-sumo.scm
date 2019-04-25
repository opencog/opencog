;; guile --no-auto-compile -l mine-sumo.scm

;; Load SUMO
;; (load "all-sumo-labeled-kb.scm")
(load "Geography.scm")

;; Set ure logger to debug
(use-modules (opencog ure))
(ure-logger-set-level! "debug")

;; Set regular logger to debug
(use-modules (opencog logger))
(cog-logger-set-level! "debug")

;; Construct corpus to mine. We select all root atoms except quanfiers
;; (ForAll, Exists, ImplicationScope, etc) statements, as they usually
;; represent rules rather than data.
(define (scope? x) (cog-subtype? 'ScopeLink (cog-type x)))
(define texts
  (filter (lambda (x) (not (scope? x))) (cog-get-all-roots)))

;; Run pattern miner
(use-modules (opencog miner))
(define results (cog-mine texts
                          #:minsup 5
                          #:maximum-iterations 500
                          #:incremental-expansion #t
                          #:max-conjuncts 2
                          #:surprisingness 'nisurp))

;; The top results are very abstract, but some are interesting, such as
;;
;; (EvaluationLink (stv 0.98404255 1)
;;    (PredicateNode "isurp")
;;    (ListLink
;;       (LambdaLink
;;          (VariableNode "$PM-229da880")
;;          (AndLink
;;             (InheritanceLink
;;                (VariableNode "$PM-229da880")
;;                (ConceptNode "SaltWaterArea" (stv 0.01 1))
;;             )
;;             (InheritanceLink
;;                (VariableNode "$PM-229da880")
;;                (ConceptNode "MaritimeClaimArea" (stv 0.01 1))
;;             )
;;          )
;;       )
;;       (ConceptNode "texts-438037533-1Ip13XKsMBkEBFng")
;;    )
;; )
;;
;; Which is rather obvious to a human who has the appropriate
;; background knowledge, but it highly surprising to the pattern
;; miner.
