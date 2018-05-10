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

;; Call the miner with a minimum support of 2
(cog-mine (list AB AC) 2)

;; Another way to call the miner on the entire atomspace instead of a
;; determine text set
(cog-mine (cog-atomspace) 2)
