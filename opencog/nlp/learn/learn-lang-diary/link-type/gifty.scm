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

; Given a rank, return a probability. This is hacky
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

; diary distribution
(set! tester-prob (/ 2 7))
(set! testy-prob (/ 1 7))
(set! test-prob (/ 1 7))
(set! gifter-prob (/ 1 7))
(set! gifty-prob (/ 1 7))
(set! gift-prob (/ 1 7))



; debug print
(display "Probability tester-prob ") (display tester-prob) (newline)
(display "Probability testy-prob ") (display testy-prob) (newline)
(display "Probability test-prob ") (display test-prob) (newline)

(display "Probability gifter-prob ") (display gifter-prob) (newline)
(display "Probability gifty-prob ") (display gifty-prob) (newline)
(display "Probability gift-prob ") (display gift-prob) (newline)
(newline)


; convert to notation used in diary
(define ta-prob test-prob)
(define tb-prob testy-prob)
(define tc-prob tester-prob)

(define ga-prob gift-prob)
(define gb-prob gifty-prob)
(define gc-prob gifter-prob)

; Compute entropy of a list of probability values
(define (h lst) 
   (define (lg x) (/ (log x) (log 2.0)))
	(define (plogp x)  (* x (lg x)))
	(- 0 (reduce + 0.0 (map plogp lst)))
)

; Initial case.
(define h-init-obs
	(h (list ta-prob tb-prob tc-prob ga-prob gb-prob gc-prob)))

(display "h initial observed: ") (display h-init-obs) (newline)

; Case 1.
(define gg-prob (+ gb-prob gc-prob))
(define tt-prob (+ tb-prob tc-prob))
