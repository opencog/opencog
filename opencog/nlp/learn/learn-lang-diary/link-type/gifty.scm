#! /usr/bin/env guile
!#
;
; gifty.scm
;
; Demo code, for the 12 July diary entry.
; Create a zipfian corpus, and see how the entropies work out.
; This avoids hand-calculation.

(use-modules (srfi srfi-1))


(define tester-rank 1)
(define testy-rank 2)
(define test-rank 3)
(define gifter-rank 4)
(define gifty-rank 5)
(define gift-rank 6)

; given a rank, return a probability. This is hacky
(define (zipf rank)
	(define (powie x)  (expt x -1.0))
	(define norm (reduce + 0 (map powie '(1 2 3 4 5 6))))
	(/ (powie rank) norm)
)

(define tester-prob (zipf 1))
(define testy-prob (zipf 2))
(define test-prob (zipf 3))
(define gifter-prob (zipf 4))
(define gifty-prob (zipf 5))
(define gift-prob (zipf 6))

(display "Probability tester-prob ") (display tester-prob) (newline)
(display "Probability testy-prob ") (display testy-prob) (newline)
(display "Probability test-prob ") (display test-prob) (newline)

(display "Probability gifter-prob ") (display gifter-prob) (newline)
(display "Probability gifty-prob ") (display gifty-prob) (newline)
(display "Probability gift-prob ") (display gift-prob) (newline)


