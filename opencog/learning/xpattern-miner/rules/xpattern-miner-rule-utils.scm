(use-modules (opencog exec))
(use-modules (opencog query))
(use-modules (opencog logger))
(use-modules (opencog rule-engine))
(use-modules (srfi srfi-1))

;; Given an atom created with minsup-eval, get the pattern, texts and
;; ms
(define (get-pattern minsup-g)
  (cog-outgoing-atom (gdr minsup-g) 0))
(define (get-texts minsup-g)
  (cog-outgoing-atom (gdr minsup-g) 1))
(define (get-ms minsup-g)
  (cog-outgoing-atom (gdr minsup-g) 2))

(define (shallow-abstraction-eval shabs-list minsup-g)
  (Evaluation
    (Predicate "shallow-abstraction")
    (List
      shabs-list
      minsup-g)))
