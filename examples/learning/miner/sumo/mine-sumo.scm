;; Make sure you disable compiling
;;
;; guile --no-auto-compile -l mine-sumo.scm

;; Load modules
(use-modules (opencog miner))
(use-modules (opencog ure))
(use-modules (opencog logger))

;; Load SUMO
;; (load "all-sumo-labeled-kb.scm")
;; (load "Geography.scm")
(load "scm/mondial.scm")

;; Set loggers
(ure-logger-set-level! "debug")
;; (ure-logger-set-timestamp! #f)
;; (ure-logger-set-sync! #t)
;; (cog-logger-set-timestamp! #f)
(cog-logger-set-level! "debug")
;; (cog-logger-set-sync! #t)

;; Set random seed
(use-modules (opencog randgen))
(cog-randgen-set-seed! 0)

;; Construct corpus to mine. We select all root atoms except quanfiers
;; (ForAll, Exists, ImplicationScope, etc) statements, as they usually
;; represent rules rather than data.
(define (scope? x) (cog-subtype? 'ScopeLink (cog-type x)))
(define texts
  ;; (filter (lambda (x) (not (scope? x))) (cog-get-all-roots)))
  ;; (filter scope? (cog-get-all-roots)))
  (cog-get-all-roots))

;; Build texts concept
(define texts-cpt (fill-texts-cpt (Concept "sumo-texts") texts))

;; Run pattern miner
(define results (cog-mine texts-cpt
                          #:minsup 100
                          #:maximum-iterations 10
                          #:incremental-expansion #t
                          #:max-conjuncts 2
                          #:surprisingness 'nisurp))
                          ;; #:surprisingness 'none))

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
