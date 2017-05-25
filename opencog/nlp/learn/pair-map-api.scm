;
; pair-map-api.scm
;
; Provides support for multi-vectors and maps for pairs.
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
; (define pma (add-pair-map pca))
; (define pdi (add-pair-support-compute pma))
; (pdi 'left-support (list (Word "the") (Word "a")))

(define-public (add-pair-map LLOBJ)
"
  add-pair-map LLOBJ - Extend LLOBJ with ability to take vector
  differences.

"
	(let ((llobj LLOBJ)
			(stars-obj (add-pair-stars LLOBJ)))

		; ---------------
		; Return the set-union of all atoms that might be paired
		; with one of the atoms from LIST on the right.
		(define (get-left-union LIST)
			(delete-duplicates!
				(append-map!
					(lambda (item) (map! gar (llobj 'left-stars item)))
					LIST)))

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

		(define (left-star-union LIST)
	)

		; Same as above, but for the right
		(define (right-star-union LIST) #f)

		; ---------------
		(define (get-pair PAIR)
		)
		(define (map-left FN LIST)
			(FN LIST)
		)

		; ---------------

	; Methods on this class.
	(lambda (message . args)
		(case message
			((left-stars)      (apply left-star-union args))
			((right-stars)     (apply right-star-union args))
			(else (apply llobj (cons message args))))
		)))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
