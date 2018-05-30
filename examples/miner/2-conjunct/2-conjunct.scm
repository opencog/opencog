;; Load the miner module
(use-modules (opencog miner))

;; (use-modules (opencog logger))
;; (cog-logger-set-level! (cog-ure-logger) "debug")
;; (cog-logger-set-level! "debug")

;; Setup the KB
(define AB
(Inheritance
  (Concept "A")
  (Concept "B")))
(define BC
(Inheritance
  (Concept "B")
  (Concept "C")))
(define DE
(Inheritance
  (Concept "D")
  (Concept "E")))
(define EF
(Inheritance
  (Concept "E")
  (Concept "F")))

;; Call the pattern miner with a minimum support of 2, starting with
;; an initial pattern with 2 conjuncts.
;;
;; Expect to learn, amonst others, the following pattern
;;
;; (Lambda
;;   (VariableList
;;     (Variable "$X")
;;     (Variable "$Y")
;;     (Variable "$Z"))
;;   (And
;;     (Inheritance
;;       (Variable "$X")
;;       (Variable "$Y"))
;;     (Inheritance
;;       (Variable "$Y")
;;       (Variable "$Z"))))
;;
;; TODO: the following bugs for no reason
;; (cog-mine (cog-atomspace) 2 #:initpat (conjunct-pattern 2))
(cog-mine (list AB BC DE EF) 2 #:initpat (conjunct-pattern 2))
