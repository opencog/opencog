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


; (define tester-rank 1)
; (define testy-rank 2)
; (define test-rank 3)
; ; (define gifter-rank 4)
; ; (define gifty-rank 5)
; ; (define gift-rank 6)
; (define gift-rank 4)
; (define gifty-rank 5)
; (define gifter-rank 6)

(define tester-rank 1)
(define gifter-rank 2)
(define testy-rank 3)
(define gifty-rank 4)
(define test-rank 5)
(define gift-rank 6)

; Given a rank, return a probability. This is hacky
(define (zipf rank)
	(define (powie x)  (expt x 0.0))
	; (define (powie x)  (expt x -1.0))
	; (define (powie x)  (expt x -1.05))
	; (define (powie x)  (expt x -1.5))
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
; (set! tester-prob (/ 2 7))
; (set! testy-prob (/ 1 7))
; (set! test-prob (/ 1 7))
; (set! gifter-prob (/ 1 7))
; (set! gifty-prob (/ 1 7))
; (set! gift-prob (/ 1 7))



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

; -------------------------------------------------------------
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

; -------------------------------------------------------------
; Case 2.
(define gac-prob (+ gc-prob ga-prob))
(define tac-prob (+ tc-prob ta-prob))

(define h-case2-obs
	(h (list tb-prob tac-prob gb-prob gac-prob)))

(define w-case2-obs
	(hw (list =ter-prob =t-prob)))

(format #t "case 2 obs h: ~f w: ~f tot: ~f ~%~%"
    h-case2-obs w-case2-obs (+ h-case2-obs w-case2-obs))

; Case 2.a.
(define g2-prob (+ gb-prob gac-prob))
(define t2-prob (+ tb-prob tac-prob))

(define h-case2a-obs
	(h (list t2-prob g2-prob)))

(define w-case2a-obs
	(hw (list =t-prob =ty-prob =ter-prob)))

(format #t "case 2a obs h: ~f w: ~f tot: ~f ~%~%"
    h-case2a-obs w-case2a-obs (+ h-case2a-obs w-case2a-obs))

; Case 2.b.
(define eb-prob (+ gb-prob tb-prob))
(define eac-prob (+ gac-prob tac-prob))

(define h-case2b-obs
	(h (list eb-prob eac-prob)))

(define w-case2b-obs
	(+ (hw (list =ter-prob =t-prob))
	   (hw (list gif-prob tes-prob))))

(format #t "case 2b obs h: ~f w: ~f tot: ~f ~%~%"
    h-case2b-obs w-case2b-obs (+ h-case2b-obs w-case2b-obs))

; -------------------------------------------------------------
; Case 3.
(define gab-prob (+ gb-prob ga-prob))
(define tab-prob (+ tb-prob ta-prob))

(define h-case3-obs
	(h (list tc-prob tab-prob gc-prob gab-prob)))

(define w-case3-obs
	(hw (list =ty-prob =t-prob)))

(format #t "case 3 obs h: ~f w: ~f tot: ~f ~%~%"
    h-case3-obs w-case3-obs (+ h-case3-obs w-case3-obs))

; Case 3.a.
(define g3-prob (+ gc-prob gab-prob))
(define t3-prob (+ tc-prob tab-prob))

(define h-case3a-obs
	(h (list t3-prob g3-prob)))

(define w-case3a-obs
	(hw (list =t-prob =ty-prob =ter-prob)))

(format #t "case 3a obs h: ~f w: ~f tot: ~f ~%~%"
    h-case3a-obs w-case3a-obs (+ h-case3a-obs w-case3a-obs))

; Case 3.b.
(define ec-prob (+ gc-prob tc-prob))
(define eab-prob (+ gab-prob tab-prob))

(define h-case3b-obs
	(h (list ec-prob eab-prob)))

(define w-case3b-obs
	(+ (hw (list =ty-prob =t-prob))
	   (hw (list gif-prob tes-prob))))

(format #t "case 3b obs h: ~f w: ~f tot: ~f ~%~%"
    h-case3b-obs w-case3b-obs (+ h-case3b-obs w-case3b-obs))

; -------------------------------------------------------------
; Case 4.
(define a-prob (+ ga-prob ta-prob))
(define b-prob (+ gb-prob tb-prob))
(define c-prob (+ gc-prob tc-prob))

(define h-case4-obs
	(h (list a-prob b-prob c-prob)))

(define w-case4-obs
	(hw (list gif-prob tes-prob)))

(format #t "case 4 obs h: ~f w: ~f tot: ~f ~%~%"
    h-case4-obs w-case4-obs (+ h-case4-obs w-case4-obs))

; Case 4.a.
(define bc-prob (+ b-prob c-prob))

(define h-case4a-obs
	(h (list a-prob bc-prob)))

(define w-case4a-obs
	(+ (hw (list gif-prob tes-prob))
	   (hw (list =ty-prob =ter-prob))))

(format #t "case 4a obs h: ~f w: ~f tot: ~f ~%~%"
    h-case4a-obs w-case4a-obs (+ h-case4a-obs w-case4a-obs))

; Case 4.b.
(define ac-prob (+ a-prob c-prob))

(define h-case4b-obs
	(h (list b-prob ac-prob)))

(define w-case4b-obs
	(+ (hw (list gif-prob tes-prob))
	   (hw (list =t-prob =ter-prob))))

(format #t "case 4b obs h: ~f w: ~f tot: ~f ~%~%"
    h-case4b-obs w-case4b-obs (+ h-case4b-obs w-case4b-obs))

; Case 4.c.
(define ab-prob (+ b-prob a-prob))

(define h-case4c-obs
	(h (list c-prob ab-prob)))

(define w-case4c-obs
	(+ (hw (list gif-prob tes-prob))
	   (hw (list =t-prob =ty-prob))))

(format #t "case 4c obs h: ~f w: ~f tot: ~f ~%~%"
    h-case4c-obs w-case4c-obs (+ h-case4c-obs w-case4c-obs))


; -------------------------------------------------------------
; Case Final.
(define ll-prob (+ ea-prob em-prob))

(define h-case-fin-obs
	(h (list ll-prob)))

(define w-case-fin-obs
	(+ (hw (list =t-prob =ty-prob =ter-prob))
	   (hw (list gif-prob tes-prob))))

(format #t "case final obs h: ~f w: ~f tot: ~f ~%~%"
    h-case-fin-obs w-case-fin-obs (+ h-case-fin-obs w-case-fin-obs))



