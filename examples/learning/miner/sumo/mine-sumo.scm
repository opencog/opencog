;; guile --no-auto-compile -l mine-sumo.scm

;; Load SUMO
(load "all-sumo-labeled-kb.scm")

;; Set ure logger to debug
(use-modules (opencog ure))
(ure-logger-set-level! "debug")

;; Run pattern miner
(use-modules (opencog miner))
(define results (cog-mine (cog-atomspace)
                          #:minsup 5000
                          #:maximum-iterations 10
                          #:incremental-expansion #t
                          #:max-conjuncts 2
                          #:surprisingness 'nisurp))
