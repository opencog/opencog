
(use-modules (rnrs bytevectors))
(use-modules (rnrs io ports))

(define (write-flo filename lst fn)

	(define xsz (length lst))
	(define ysz (length lst))
	(define nrow 0)
	(define flo (open-file filename "w"))
	(format flo "~d ~d\n" xsz ysz)
	(for-each
		(lambda (row)
			(define ncol 0)
			; Make room for IEE floats
			(define bvrow (make-bytevector (* 4 xsz)))
			(for-each
				(lambda (col)
					(define number (fn row col))
					(bytevector-ieee-single-native-set! bvrow ncol number)
					(set! ncol (+ ncol 1))
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

