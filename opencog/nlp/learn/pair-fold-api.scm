;
; pair-fold-api.scm
;
; Provides support for folding down tuples of pairs.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; There is sometimes a need to take differences and sums of vectors,
; or apply other kinds of transforms.  This provides eh API to do that.
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))

; ---------------------------------------------------------------------
;
; Example usage:
; (define (subtract TUPLE)  (- (first TUPLE) (second TUPLE)))
; (define pma (add-tuple-math pca subtract))
; (define pdi (add-pair-support-compute pma))
; (pdi 'right-support (list (Word "the") (Word "a")))

(define-public (add-tuple-math LLOBJ FUNC)
"
  add-tuple-math LLOBJ FUNC - Extend LLOBJ with ability to take
  tuples of items, and then call FUNC on that tuple, whenever
  the 'pair-count method is invoked.
"
	(let ((llobj LLOBJ)
			(star-obj (add-pair-stars LLOBJ))
			(sum-func FUNC))

		; ---------------
		; Return the set-union of all atoms that might be paired
		; with one of the atoms from TUPLE on the right.
		(define (get-left-union TUPLE)
			(delete-duplicates!
				(append-map!
					(lambda (item) (map! gar (star-obj 'left-stars item)))
					TUPLE)))

		(define (get-right-union TUPLE)
			(delete-duplicates!
				(append-map!
					(lambda (item) (map! gdr (star-obj 'right-stars item)))
					TUPLE)))

		; ---------------
		; Given a TUPLE of items of 'right-type, this returns
		; a tuple of low-level pairs of LEFTY and each righty
		; in the TUPLE.  If such a pair does not exist, the
		; returned tuple will contain an empty list at that locus.
		(define (get-left-lopr-tuple LEFTY TUPLE)
			(define prty (llobj 'pair-type))
			(map
				(lambda (rght) (cog-link prty LEFTY rght))
				TUPLE))

		(define (get-right-lopr-tuple RIGHTY TUPLE)
			(define prty (llobj 'pair-type))
			(map
				(lambda (left) (cog-link prty left RIGHTY))
				TUPLE))

		; ---------------
		; Expects TUPLE to be a scheme list of items of 'right-type.
		; Returns a list of tuples of the left-stars appropriae for
		; that TUPLE.  The left-star tuples are "aligned", so that
		; that within one tuple, all pairs have exactly the same
		; left side.  If such a pair does not exist, the empty list
		; stands in its place.  The only case where this would not
		; happen is if all items in the TUPLE had exactly the same
		; left-wilds.  But this would be a very unusual thing, in the
		; normal case.
		;
		; So, for example, if TUPLE is (list (Word "the") (Word "a"))
		; This might return a list of say, 3 tuples:
		;   (list
		;      (list  ; Note left atoms are identical.
		;          (ListLink (Word "foo") (Word "the"))
		;          (ListLink (Word "foo") (Word "a")))
		;      (list
		;          '()  ; Note the pair bar:the does not exist
		;          (ListLink (Word "bar") (Word "a")))
		;      (list
		;          (ListLink (Word "zed") (Word "the"))
		;          '()))  ; Note the par zed-a does not exist.
		;
		; In the above, the union of the left support was {foo, bar, zed}
		; and the intersection of the left support was just {foo}.  This
		; will be the typical case: the intersection will be typically
		; non-empty, and hethe union will typically be strictly alrger.

		(define (left-star-union TUPLE)
			(map
				(lambda (lefty) (get-left-lopr-tuple lefty TUPLE))
				(get-left-union TUPLE)))

		; Same as above, but for the right
		(define (right-star-union TUPLE)
			(map
				(lambda (righty) (get-right-lopr-tuple righty TUPLE))
				(get-right-union TUPLE)))

		; ---------------
		; Given a TUPLE of low-level pairs, return a tuple of high-level
		; pairs.
		(define (get-pair TUPLE)
			(map (lambda (lopr) (llobj 'item-pair lopr)) TUPLE))

		; Given a TUPLE of high-level pairs, return a single number.
		; The sum-func is applied to reduce the counts on each pair
		; in the tuple down to just one number.
		(define (get-func-count TUPLE)
			(sum-func
				(map
					(lambda (pr)
						(if (null? pr) 0 (llobj 'pair-count pr)))
					TUPLE)))

		; ---------------
		; Return a pointer to each method that this class overloads.
		(define (provides meth)
			(case meth
				((left-stars) left-star-union)
				((right-stars) right-star-union)
				((item-pair) get-pair)
				((pair-count) get-func-count)
				(else #f)))

		; ---------------

	; Methods on this class.
	(lambda (message . args)
		(case message
			((left-stars)      (apply left-star-union args))
			((right-stars)     (apply right-star-union args))
			((item-pair)       (apply get-pair args))
			((pair-count)      (apply get-func-count args))
			((provides)        (apply provides args))
			(else (apply llobj (cons message args))))
		)))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
