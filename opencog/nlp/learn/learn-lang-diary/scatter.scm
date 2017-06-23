
(use-modules (ice-9 format))
(use-modules (rnrs bytevectors))
(use-modules (rnrs io ports))

(define (write-flo filename lst fn)

	(define xsz (length lst))
	(define ysz (length lst))
	(define nrow 0)
	(define flo (open-file filename "w"))
	; Should always be 24 bytes, including the null-terminator
	; Appending JUNK because otherwise the print screws up somehow.
	(format flo "~7d~7d\nJUNK" xsz ysz)
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
(format #t "duuude its this: ~A\n" bvrow)
(set! ncol 0)
(for-each
	(lambda (col)
	(format #t "duude its ~A\n" (bytevector-ieee-single-native-ref bvrow ncol))
	(set! ncol (+ ncol 4))
	)
	lst)
			(force-output flo)
			(set! nrow (+ nrow 1))
			(format #t "Done with row ~d of ~d\n" nrow ysz)
		)
		lst)
	(close-port flo)
)

