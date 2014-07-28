;
; gifty.scm
;
; Demo code, for the 12 July diary entry.
; Create a zipfian corpus, and see how the entropies work out.
; This avoids hand-calculation.

(define tester-rank 1)
(define testy-rank 2)
(define test-rank 3)
(define gifter-rank 4)
(define gifty-rank 5)
(define gift-rank 6)

; given a rank, return a probability. This is hacky
(define (zipf rank)
    (/ 1.0 6.0)
)

(define tester-prob (zipf 1))
(define testy-prob (zipf 2))
(define test-prob (zipf 3))
(define gifter-prob (zipf 4))
(define gifty-prob (zipf 5))
(define gift-prob (zipf 6))

(display "Probability tester-prob ") (display tester-prob) (newline)


