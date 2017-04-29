

; Make a histogram of the left-right imbalance of word-pair counts.
; This is graphed in the diary.
(define (mkhist)
   (define nbins 300)
   (define lo -1.0)
   (define hi 1.0)
   (define inc (/ nbins (- hi lo)))

	; the histogram
   (define hist (make-array 0 nbins))

	; the left-right asymmetry
   (define (asym w) (/ (- (get-right-count-str w) (get-left-count-str w))
		(get-word-count-str w)))

	; Bin-count
   (for-each (lambda (w)
         (define bin (round (* inc (- (asym (cog-name w)) lo))))
         
         (if (nan? bin) (set! bin 0))
            (set! bin (inexact->exact bin))
         (if (>= bin nbins) (set! bin (- nbins 1)))
         (if (< bin 0) (set! bin 0))
         (array-set! hist (+ 1 (array-ref hist bin)) bin))
      (cog-get-atoms 'WordNode))

	; Print the bin contents
   (array-for-each (lambda (cnt)
         (format #t "~A ~A\n" x cnt)
         (set! x (+ x (/ 1.0 inc))))
      hist)
)
