;; Load the miner module
(use-modules (opencog)
             (opencog miner))

;; Setup the KB
(define AB
(Inheritance
  (Concept "A")
  (Concept "B")))
(define AC
(Inheritance
  (Concept "A")
  (Concept "C")))

;; Call the miner on the entire atomspace with minimum support of 2
;;
;; Expect to learn, among others, the following pattern
;;
;; (Lambda
;;   (Variable "$X")
;;   (Present
;;     (Inheritance
;;       (Concept "A")
;;       (Variable "$X"))))
;;
;; Since the pattern has no conjunction the surprisingness measure is
;; none (as the current surprisingness measures consider the
;; surprisingness of conjunctions of components)
(define results-as (cog-mine (cog-atomspace)
                             #:minsup 2
                             #:incremental-expansion #f
                             #:surprisingness 'none))

;; Call the miner on the text set of interest instead, should yield
;; the same results.
(define results-lst (cog-mine (list AB AC)
                              #:minsup 2
                              #:incremental-expansion #f
                              #:surprisingness 'none))
