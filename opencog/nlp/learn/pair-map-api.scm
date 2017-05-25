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
		; with one of the atoms from TUPLE on the right.
		(define (get-left-union TUPLE)
			(delete-duplicates!
				(append-map!
					(lambda (item) (map! gar (llobj 'left-stars item)))
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
