;
; pair-stats.scm
;
; Return assorted database statistics, pertaining to word-pairs.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; The statistics reported here are those collected via the code
; in `link-pipeline.scm` and computed in `compute-mi.scm`.  Therefore
; structure defintions there and here need to be maintained
; consistently.
;
; Many or most of the stats returned here assume that the pair-counting
; batch job has completed. They get stats based on the current contents
; of the atomspace.
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------

(define-public (add-pair-mi-api LLOBJ)
"
  add-pair-mi-api LLOBJ - methods for MI and entropy of pairs.

  Extend the LLOBJ with additional methods to compute the one-sided
  entropies and mutual information of pairs.

  The object must have valid pair-frequency values on it, accessible
  via the standard frequency-object API. These must have been
  pre-computed, before this object can be used.
"
	; Need the 'left-stars method, provided by add-pair-wildcards
	; Need the 'left-wild-freq method, provided by add-pair-freq-api
	(let ((frqobj (add-pair-freq-api (add-pair-wildcards LLOBJ))))

		; Compute the left-wild entropy summation:
		;    h_left(y) = -sum_x P(x,y) log_2 P(x,y)
		;
		; Note that
		;    h_total = sum_y h_left(y)
		(define (compute-left-entropy RIGHT-ITEM)
			(fold
				(lambda (PAIR sum) (+ sum (frqobj 'pair-entropy PAIR)))
				0
				(frqobj 'left-stars RIGHT-ITEM)))

		; Compute the right-wild entropy summation:
		;    h_right(x) = -sum_y P(x,y) log_2 P(x,y)
		;
		; Note that
		;    h_total = sum_x h_right(x)
		(define (compute-right-entropy LEFT-ITEM)
			(fold
				(lambda (PAIR sum) (+ sum (frqobj 'pair-entropy PAIR)))
				0
				(frqobj 'right-stars LEFT-ITEM)))

		; Compute the left-fractional entropy summation:
		;    H_left(y) = h_left(y) / P(*,y)
		; Note that
		;    h_total = sum_y P(*,y) H_left(y)
		(define (compute-left-fractional RIGHT-ITEM)
			(/ (compute-left-entropy RIGHT-ITEM)
				(frqobj 'left-wild-freq RIGHT-ITEM)))

		; As above, but flipped.
		(define (compute-right-fractional LEFT-ITEM)
			(/ (compute-right-entropy LEFT-ITEM)
				(frqobj 'right-wild-freq LEFT-ITEM)))

		; Methods on this class.
		(lambda (message . args)
			(case message
				((compute-left-entropy)  (apply compute-left-entropy args))
				((compute-right-entropy) (apply compute-right-entropy args))
				((compute-left-fractional)  (apply compute-left-fractional args))
				((compute-right-fractional) (apply compute-right-fractional args))
				(else (apply frqobj      (cons message args))))
		))
)

; ---------------------------------------------------------------------

(define-public (add-total-entropy-api FRQOBJ)
"
  add-total-entropy-api FRQOBJ - methods for total and partial entropy.

  Extend the FRQOBJ with additional methods to compute the partial
  and total entropies of the total set of pairs.

  The FRQOBJ needs to be an object implementing methods to get pair
  observation frequencies, which must return valid values; i.e. must
  have been previously computed. Specifically, it must have the
  'left-logli, 'right-logli and 'pair-logli methods.

  These methods loop over all pairs, and so can take a lot of time.
"
	(let ((frqobj FRQOBJ))

		; Compute the total entropy for the set. This loops over all
		; pairs, and computes the sum
		;   H_tot = sum_x sum_y p(x,y) log_2 p(x,y)
		; It returns a single numerical value, for the entire set.
		(define (compute-total-entropy)
			(define entropy 0)

			(define (right-loop left-item)
				(for-each
					(lambda (lipr)
						; The get-logli below may throw an exception, if
						; the particular item-pair doesn't have any counts.
						; XXX does this ever actually happen?  It shouldn't,
						;right?
						(catch #t (lambda ()
								(define pr-freq (frqobj 'pair-freq lipr))
								(define pr-logli (frqobj 'pair-logli lipr))
								(define h (* pr-freq pr-logli))
								(set! entropy (+ entropy h))
							)
							(lambda (key . args) #f))) ; catch handler
					(frqobj 'right-stars left-item)))

			(for-each right-loop (frqobj 'left-support))

			; Return the single number.
			entropy
		)

		; Compute the left-wildcard partial entropy for the set. This
		; loops over all left-wildcards, and computes the sum
		;   H_left = sum_y p(*,y) log_2 p(*,y)
		; It returns a single numerical value, for the entire set.
		(define (compute-left-entropy)

			(fold
				(lambda (right-item sum)
					(+ sum (frqobj 'left-wild-logli right-item)))
				0
				(frqobj 'right-support))
		)

		; Compute the right-wildcard partial entropy for the set. This
		; loops over all right-wildcards, and computes the sum
		;   H_right = sum_x p(x,*) log_2 p(x,*)
		; It returns a single numerical value, for the entire set.
		(define (compute-right-entropy)

			(fold
				(lambda (left-item sum)
					(+ sum (frqobj 'right-wild-logli left-item)))
				0
				(frqobj 'left-support))
		)

		; Methods on this class.
		(lambda (message . args)
			(case message
				((total-entropy)         (compute-total-entropy))
				((left-entropy)          (compute-left-entropy))
				((right-entropy)         (compute-right-entropy))
				(else (apply frqobj      (cons message args))))
		))
)

; ---------------------------------------------------------------------
