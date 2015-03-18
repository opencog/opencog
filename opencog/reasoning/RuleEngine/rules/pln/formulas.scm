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
