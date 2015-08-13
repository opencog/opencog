; =============================================================================
; A list of the helper functions necessary in different inference rules
; -----------------------------------------------------------------------------
; Index
; - inversion-formula
; - simple-deduction-formula
; - find-replace
;------------------------------------------------------------------------------


; =============================================================================
; Inversion Formula
; sBA = (sAB * sB)/sA
; -----------------------------------------------------------------------------

(define (inversion-formula sAB sA sB)
	(/ (* sAB sB) sA))

; =============================================================================
; Simple Deduction Formula
; IF: 0 <= sAB-max((sA+sB-1)/sA, 0) 0<= min(1, sB/sA) -max((sA+sB-1)/sA, 0)
; AND 0 <= sBC-max((sB+sC-1)/sB, 0) 0<= min(1, sC/sB)-max((sB+sC-1)/sB, 0)
; sAC = [sAB.sBC + ((1-sAB)(sC - sBsBC))/(1-sB)] 
; ELSE:
; sAC = 0
; -----------------------------------------------------------------------------

; Consistency Conditions
(define (consistency1 sA sB sAB)
	(- sAB
		(max 
			(/ 
				(- 
					(+ sA sB) 
					1) 
				sA) 
			0)))

(define (consistency2 sA sB sAB)
	(- 
		(min 
			1 
			(/ sB sA)) 
		sAB))

; Main Formula

(define (simple-deduction-formula sA sB sC sAB sBC)
	(if
		(and
				(>= (consistency1 sA sB sAB) 0)
				(>= (consistency2 sA sB sAB) 0)
				(>= (consistency1 sB sC sBC) 0)
				(>= (consistency2 sB sC sBC) 0))
		(+ 
			(* sAB sBC) 
			(/ 
				(* 
					(- 1 sAB) 
					(- sC 
						(* sB sBC))) 
				(- 1 sB)))
		0 ))

; =============================================================================
; Basic find and replace formula
;
; Returns a new list by replacing first occurance of an element in the 
; list(here old) to another element in the list(here new).
; -----------------------------------------------------------------------------

(define (find-replace l old new)
	(cond
		((null? l) '())
		((equal? (car l) old) (cons new (cdr l)))
		(else
			(cons (car l) (find-replace (cdr l) old new)))))

; =============================================================================
; Invert formula
;
; Inverts the input
; -----------------------------------------------------------------------------

(define (invert a)
    (/ 1.0 a))

; =============================================================================
; Negate formula
;
; Negates the probability
; -----------------------------------------------------------------------------

(define (negate a)
    (- 1 a))

; =============================================================================
; Transitive Similarity Formula
;
; Returns the strength value of the transitive similarity rule
; -----------------------------------------------------------------------------

(define (transitive-similarity-formula sA sB sC sAB sBC )
    (let
        ((T1 (/ (* (+ 1 (/ sB sA)) (sAB)) (+ 1 sAB)))
         (T2 (/ (* (+ 1 (/ sC sB)) (sBC)) (+ 1 sBC)))
         (T3 (/ (* (+ 1 (/ sB sC)) (sBC)) (+ 1 sBC)))
         (T4 (/ (* (+ 1 (/ sA sB)) (sAB)) (+ 1 sAB))))
        (invert (- (+ (invert (+ (* T1 T2) (* (negate T1) (/ (- sC (* sB T2)) (negate sB)))))
                      (invert (+ (* T3 T4) (* (negate T3) (/ (- sC (* sB T4)) (negate sB)))))) 1))))

; =============================================================================
; PreciseModusPonens formula
;
; Returns the strength value of the precise modus ponens rule
; -----------------------------------------------------------------------------

(define (precise-modus-ponens-formula sA sAB snotAB)
    (+ (* sAB sA) (* snotAB (negate sA))))

