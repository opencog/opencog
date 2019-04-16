;; guile --no-auto-compile -l mine-sumo.scm

;; Load SUMO
;; (load "scm/all-sumo-labeled-kb.scm")
;; (load "scm/Merge.scm")
;; (load "scm/Music.scm")
;; (load "scm/Geography.scm")
(load "scm/WorldAirports.scm")

;; Set ure logger to debug
(use-modules (opencog ure))
(ure-logger-set-level! "debug")

;; Run pattern miner
(use-modules (opencog miner))
(define results (cog-mine (cog-atomspace)
                          #:minsup 100
                          #:maximum-iterations 100
                          #:incremental-expansion #t
                          #:max-conjuncts 2
                          #:surprisingness 'nisurp))
