;; =============================================================================
;; Negate formula
;;
;; Negates the probability
;; -----------------------------------------------------------------------------

(define (negate a)
    (- 1 a))

;; =============================================================================
;; PreciseModusPonens formula
;;
;; Returns the strength value of the precise modus ponens rule
;; -----------------------------------------------------------------------------

(define (precise-modus-ponens-strength-formula sA sAB snotAB)
    (+ (* sAB sA) (* snotAB (negate sA))))

