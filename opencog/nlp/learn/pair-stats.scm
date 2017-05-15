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

(define-public (add-total-entropy-api FRQOBJ)
"
  add-total-entropy-api FRQOBJ - methods for total and partial entropy.

  Extend the FRQOBJ with additional methods to compute the partial
  and total entropies of the total set of pairrs.

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
; ---------------------------------------------------------------------

(define (count-clique-pair PAIR)
"
  Return count for the clique-pair PAIR
  PAIR should be a ListLink of two words.
"
	(define evl (cog-link 'EvaluationLink pair-pred PAIR))
	(if (null? evl) 0 (get-count evl))
)

(define (count-dist-pair PAIR)
"
  Return sum over all counts of the distance pairs.
  This should, in all cases, return the same value as
  `count-clique-pair`, above, since the sum over distances
  should equal the total number of observations. This can be
  checked with the `verify-clique-pair-sums` function below.

  PAIR should be a ListLink of two words.
"
	(fold
		(lambda (ex sum) (+ (get-count ex) sum))
		0
		(filter
			(lambda (lnk) (equal? pair-dist (gar lnk)))
			(cog-incoming-by-type PAIR 'ExecutionLink)))
)

; ---------------------------------------------------------------------

(define-public (verify-clique-pair-sums PAIR-LIST)
"
  This checks consistency of the the clique-pair total count, with
  the subcounts of each pair, accodring to the distance between
  the words. The sum of the subtotals should equal the total.
  It should not throw.

  Example usage: (verify-clique-pair-sums (get-all-clique-pairs))
"
	(define cnt 0)
	(for-each
		(lambda (PAIR)
			(set! cnt (+ cnt 1))
			(if (not (eqv? (count-dist-pair PAIR) (count-clique-pair PAIR)))
				(throw 'bad-count 'foobar PAIR)
				(format #t "Its OK ~A\n" cnt)
			))
		PAIR-LIST)
)

; ---------------------------------------------------------------------
