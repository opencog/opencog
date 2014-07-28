#! /usr/bin/env guile
!#
;
; gifty.scm
;
; Demo code, for the 12 July diary entry.
; Create a zipfian corpus, and see how the entropies work out.
; This avoids hand-calculation.

(use-modules (srfi srfi-1))
(use-modules (ice-9 format))


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


; Convert to notation used in diary -- these are the link frequencies
(define ta-prob test-prob)
(define tb-prob testy-prob)
(define tc-prob tester-prob)

(define ga-prob gift-prob)
(define gb-prob gifty-prob)
(define gc-prob gifter-prob)

; We also want the morpheme (word) frequencies.
(define gif-prob (+ gift-prob gifty-prob gifter-prob))
(define tes-prob (+ test-prob testy-prob tester-prob))
(define =ter-prob (+ gifter-prob tester-prob))
(define =ty-prob  (+ gifty-prob testy-prob))
(define =t-prob   (+ gift-prob test-prob))

; Compute entropy of a list of probability values
(define (h lst)
   (define (lg x) (/ (log x) (log 2.0)))
	(define (plogp x)  (* x (lg x)))
	(- 0 (reduce + 0.0 (map plogp lst)))
)
; Like the above, but explicitly normalize the list of probabilities
; That is, the input list is first normalized so tht it sums to 1.0
(define (hw lst)
	(define norm (reduce + 0.0 lst))
	(define nlist (map (lambda (x) (/ x norm)) lst))
	(h nlist)
)

; Initial case.
(define h-init-obs
	(h (list ta-prob tb-prob tc-prob ga-prob gb-prob gc-prob)))
(display "h initial observed: ") (display h-init-obs) (newline)
(newline)

; Case 1.
(define gg-prob (+ gb-prob gc-prob))
(define tt-prob (+ tb-prob tc-prob))

(define h-case1-obs
	(h (list ta-prob tt-prob ga-prob gg-prob)))

(define w-case1-obs
	(hw (list =ty-prob =ter-prob)))

(format #t "case 1 obs h: ~f w: ~f tot: ~f ~%~%"
    h-case1-obs w-case1-obs (+ h-case1-obs w-case1-obs))

; Case 1.a.
(define g-prob (+ ga-prob gg-prob))
(define t-prob (+ ta-prob tt-prob))

(define h-case1a-obs
	(h (list t-prob g-prob)))

(define w-case1a-obs
	(hw (list =t-prob =ty-prob =ter-prob)))

(format #t "case 1a obs h: ~f w: ~f tot: ~f ~%~%"
    h-case1a-obs w-case1a-obs (+ h-case1a-obs w-case1a-obs))

; Case 1.b.
(define ea-prob (+ ga-prob ta-prob))
(define em-prob (+ gg-prob tt-prob))

(define h-case1b-obs
	(h (list ea-prob em-prob)))

(define w-case1b-obs
	(+ (hw (list =ty-prob =ter-prob))
	   (hw (list gif-prob tes-prob))))

(format #t "case 1b obs h: ~f w: ~f tot: ~f ~%~%"
    h-case1b-obs w-case1b-obs (+ h-case1b-obs w-case1b-obs))

; Case Final.
(define ll-prob (+ ea-prob em-prob))

(define h-case-fin-obs
	(h (list ll-prob)))

(define w-case-fin-obs
	(+ (hw (list =t-prob =ty-prob =ter-prob))
	   (hw (list gif-prob tes-prob))))

(format #t "case final obs h: ~f w: ~f tot: ~f ~%~%"
    h-case-fin-obs w-case-fin-obs (+ h-case-fin-obs w-case-fin-obs))



