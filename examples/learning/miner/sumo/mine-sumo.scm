;; guile --no-auto-compile -l mine-sumo.scm

;; Load SUMO
;; (load "scm/all-sumo-labeled-kb.scm")
;; (load "scm/Merge.scm")
;; (load "scm/Music.scm")
;; (load "scm/Geography.scm")
;; (load "scm/Cars.scm")
(load "scm/Hotel.scm")

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
                          #:maximum-iterations 10000
                          #:incremental-expansion #t
                          #:max-conjuncts 2
                          #:surprisingness 'nisurp))
