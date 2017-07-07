
(define (pair-sym-cache wa wb KEY FN)
"
	Get cached value, or compute it.  Symmetric.
	Stores the value in the atomspace, under KEY for (ListLink wa wb)
"
	(define (get-val PR)
		(cog-value-ref (cog-value PR KEY) 0))
	(define (set-val PR VAL)
		(cog-set-value! PR KEY (FloatValue VAL)))

	; Where to check for values
	(define wpr (ListLink wa wb))

	; The value, or #f
	(define got
		(catch #t
			(lambda () (get-val wpr))
			(lambda (k . args) #f)))

	; If the value is #f, then try the reversed pair.
	(if got got
		(let* ((flp (ListLink wb wa))
				(gat (catch #t
					(lambda () (get-val flp))
					(lambda (k . args) #f))))
			(if gat
				; Found the reversed pair; record he forward pair
				(begin
					(set-val wpr gat)
					gat)
				; Found neither foreward nor reversed. Compute it.
				(let ((val (FN wa wb)))
					(set-val wpr val)
					(set-val flp val)
					val)))))

(define (left-sum ITEM LST FN)
"
 Perform the wild-card sum over FN(*, ITEM) for * in LST
"
	(define (summer ITEM)
		(fold
			(lambda (it acc) (+ acc (FN it ITEM)))
			0
			LST))
	(make-afunc-cache summer))


