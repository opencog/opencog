
(use-modules (ice-9 format))
(use-modules (ice-9 binary-ports))
(use-modules (rnrs bytevectors))
(use-modules (rnrs io ports))

(define (write-flo filename lst fn)

	(define xsz (length lst))
	(define ysz (length lst))
	(define nrow 0)
	(define flo (open-file filename "w"))
	; Write 23 bytes, not counting the null-terminator
	(format flo "~7d ~7d" xsz ysz)
	(put-u8 flo 0)  ; Write a null byte.
	(for-each
		(lambda (row)
			(define ncol 0)
			; Make room for IEE floats
			(define bvrow (make-bytevector (* 4 xsz)))
			(for-each
				(lambda (col)
					(define number (fn row col))
					; (bytevector-ieee-single-set! bvrow ncol number 'little)
					(bytevector-ieee-single-native-set! bvrow ncol number)
					(set! ncol (+ ncol 4))
				)
				lst)
			; Now write out the row.
			(put-bytevector flo bvrow)
			(force-output flo)
			(set! nrow (+ nrow 1))
			(format #t "Done with row ~d of ~d\n" nrow ysz)
		)
		lst)
	(close-port flo)
)

