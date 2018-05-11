;; Load the miner module
(use-modules (opencog miner))

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
;; Expect to learn, amonst others, the following pattern
;;
;; (Lambda
;;   (Variable "$X")
;;   (Inheritance
;;     (Concept "A")
;;     (Variable "$X")))
(cog-mine (cog-atomspace) 2)

;; Call the miner on the text set of interest instead, should yield
;; the same results.
(cog-mine (list AB AC) 2)
