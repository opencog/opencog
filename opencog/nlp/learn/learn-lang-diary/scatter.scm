;
; scatter.scm - create scatterplot images. Write out single-channel
; "flo" images.
;
(use-modules (ice-9 format))
(use-modules (ice-9 binary-ports))
(use-modules (rnrs bytevectors))
(use-modules (rnrs io ports))

(define-public (write-flo filename LST FN)
"
 write-flo - write a single-color-channel floating-point image of a
 matrix. This assumes that FN is a function, taking two arguments,
 returning a float. The LST is assumed to be a list of basis elements
 to be passed to FN.  The resulting image consists of the x and y
 dimensions (in ASCII) followed by the floats.  The result is saved to
 `filename`.

 Example usage:
    (define psu (add-support-api psa))
    (define long-words
       (filter (lambda (word) (<= 128 (psu 'right-length word)))
          (psu 'left-basis)))

    (define (pair-cos A B)
       (define cos-key (PredicateNode \"*-Cosine Sim Key-*\"))
       (define SIM (SimilarityLink A B))
       (cog-value-ref (cog-value SIM cos-key) 0))

    (write-flo \"scat-cosine-big.flo\" long-words pair-cos)
"
	(define xsz (length LST))
	(define ysz (length LST))
	(define nrow 0)
	(define flo (open-file filename "w"))
	; Write 23 bytes, not counting the null-terminator
	(format flo "~7d ~7d" xsz ysz)
	(put-u8 flo 0)  ; Write a null byte.
	(for-each
		(lambda (row)
			(define ncol 0)
			; Make room for IEEE floats
			(define bvrow (make-bytevector (* 4 xsz)))
			(for-each
				(lambda (col)
					(define number (FN row col))
					; (bytevector-ieee-single-set! bvrow ncol number 'little)
					(bytevector-ieee-single-native-set! bvrow ncol number)
					(set! ncol (+ ncol 4))
				)
				LST)
			; Now write out the row.
			(put-bytevector flo bvrow)
			(force-output flo)
			(set! nrow (+ nrow 1))
			(format #t "Done with row ~d of ~d\n" nrow ysz)
		)
		LST)
	(close-port flo)
)
