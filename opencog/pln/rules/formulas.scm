;; =============================================================================
;; A list of the helper functions necessary in different inference rules
;; -----------------------------------------------------------------------------
;; Index
;; - simple-deduction-strength-formula
;; - inversion-strength-formula
;; - find-replace
;;------------------------------------------------------------------------------

;; =============================================================================
;; Simple Deduction Formula
;;
;; Preconditions:
;;
;;   1. Check that P(B|A) makes sense
;;
;;      1.a. P(B|A) is defined
;;
;;         0.0 < sA
;;
;;      1.b. P(B|A) is greater than or equal to P(A,B)/P(A) considering
;;      the smallest possible intersection between A and B
;;
;;         max((sA+sB-1)/sA, 0) <= sAB
;;
;;      1.c. P(B|A) is smaller than P(A,B)/P(A) considering the largest
;;      possible intersection between A and B
;;
;;         sAB <= min(1, sB/sA)
;;
;;   2. Check that P(C|B) makes sense
;;
;;      1.a. P(C|B) is defined
;;
;;         0.0 < sB
;;
;;      1.b. P(C|B) is greater than P(B,C)/P(B) considering the
;;      smallest possible intersection between B and C
;;
;;         max((sB+sC-1)/sB, 0) <= sBC
;;
;;      1.c. P(C|B) is greater than P(B,C)/P(B) considering the
;;      largest possible intersection between B and C
;;
;;         sBC <= min(1, sC/sB)
;;
;; Calculation:
;;
;;   sAC = [sAB.sBC + ((1-sAB)(sC - sBsBC))/(1-sB)]
;;
;; To avoid division by 0 (which the preconditions do not necessarily
;; prevent), i.e. when sB tends to 1 we find that
;;
;;   sAC tends to sC
;;
;; Indeed, in such case sAB tends to sB and sBC tends to sC. As a
;; result (according to the main formula above) sAC tends to
;;
;;   sB.sC + ((1-sB)(sC - sBsC))/(1-sB) = sB.sC + (sC - sB.sC) = sC
;;
;; If the preconditions are not met it returns sAC = 0, although
;; honnestly the deduction rule should not have been applied in the
;; first place.
;; -----------------------------------------------------------------------------

(use-modules (opencog logger))

; Consistency Conditions
(define (smallest-intersection-probability sA sB)
  (max (/ (+ sA sB -1) sA) 0))

(define (largest-intersection-probability sA sB)
  (min (/ sB sA) 1))

(define (conditional-probability-consistency sA sB sAB)
  (and (< 0 sA)
       (<= (smallest-intersection-probability sA sB) sAB)
       (<= sAB (largest-intersection-probability sA sB))))

; Main Formula

(define (simple-deduction-strength-formula sA sB sC sAB sBC)
  (if
     (and
        (conditional-probability-consistency sA sB sAB)
        (conditional-probability-consistency sB sC sBC))
     ;; Preconditions are met
     (if (< 0.99 sB)
        ;; sB tends to 1
        sC
        ;; otherwise
        (+ (* sAB sBC) (/ (* (- 1 sAB) (- sC (* sB sBC))) (- 1 sB))))
     ;; Preconditions are not met
     0))

;; =============================================================================
;; Inversion Formula
;;
;; Preconditions:
;;
;;   1. Check that P(B|A) makes sense
;;
;;      1.a. P(B|A) is defined
;;
;;         0.0 < sA
;;
;;      1.b. P(B|A) is greater than or equal to P(A,B)/P(A) considering
;;      the smallest possible intersection between A and B
;;
;;         max((sA+sB-1)/sA, 0) <= sAB
;;
;;      1.c. P(B|A) is smaller than P(A,B)/P(A) considering the largest
;;      possible intersection between A and B
;;
;;         sAB <= min(1, sB/sA)
;;
;;   2. Check that P(A|B) makes sense
;;
;;      1. P(A|B) is defined
;;
;;         0.0 < sB
;;
;;         And that is all, ultimately the calculation should result
;;         into a consistent sBA (or let's hope so).
;;
;; Calculation:
;;
;;   sBA = (sAB * sB) / sA
;;
;; -----------------------------------------------------------------------------

(define (inversion-consistency sA sB sAB)
  (and (< 0 sA)
       (< 0 sB)
       (<= (smallest-intersection-probability sA sB) sAB)
       (<= sAB (largest-intersection-probability sA sB))))

(define (inversion-strength-formula sA sB sAB)
  (if (inversion-consistency sA sB sAB)
      (/ (* sAB sA) sB)
      0))

;; =============================================================================
;; Basic find and replace formula
;;
;; Returns a new list by replacing first occurance of an element in the
;; list(here old) to another element in the list(here new).
;; -----------------------------------------------------------------------------

(define (find-replace l old new)
	(cond
		((null? l) '())
		((equal? (car l) old) (cons new (cdr l)))
		(else
			(cons (car l) (find-replace (cdr l) old new)))))

;; =============================================================================
;; Invert formula
;;
;; Inverts the input
;; -----------------------------------------------------------------------------

(define (invert a)
    (/ 1.0 a))

;; =============================================================================
;; Negate formula
;;
;; Negates the probability
;; -----------------------------------------------------------------------------

(define (negate a)
    (- 1 a))

;; =============================================================================
;; Transitive Similarity Formula
;;
;; Returns the strength value of the transitive similarity rule
;; -----------------------------------------------------------------------------

(define (transitive-similarity-strength-formula sA sB sC sAB sBC )
    (let
        ((T1 (/ (* (+ 1 (/ sB sA)) (sAB)) (+ 1 sAB)))
         (T2 (/ (* (+ 1 (/ sC sB)) (sBC)) (+ 1 sBC)))
         (T3 (/ (* (+ 1 (/ sB sC)) (sBC)) (+ 1 sBC)))
         (T4 (/ (* (+ 1 (/ sA sB)) (sAB)) (+ 1 sAB))))
        (invert (- (+ (invert (+ (* T1 T2) (* (negate T1) (/ (- sC (* sB T2)) (negate sB)))))
                      (invert (+ (* T3 T4) (* (negate T3) (/ (- sC (* sB T4)) (negate sB)))))) 1))))

;; =============================================================================
;; PreciseModusPonens formula
;;
;; Returns the strength value of the precise modus ponens rule
;; -----------------------------------------------------------------------------

(define (precise-modus-ponens-strength-formula sA sAB snotAB)
    (+ (* sAB sA) (* snotAB (negate sA))))

