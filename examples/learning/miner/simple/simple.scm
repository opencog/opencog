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
;;   (Inheritance
;;     (Concept "A")
;;     (Variable "$X")))
(define results (cog-mine (cog-atomspace) #:minsup 2))

;; Call the miner on the text set of interest instead, should yield
;; the same results.
; FIXME: The following fails
;(cog-mine (list AB AC) 2)
