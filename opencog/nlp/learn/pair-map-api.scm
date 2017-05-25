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

(define-public (add-pair-nap LLOBJ)
"
  add-pair-map LLOBJ - Extend LLOBJ with ability to take vector
  differences.

"
	(let ((llobj LLOBJ)
			(stars-obj (add-pair-stars LLOBJ))
		)

		(define (map-left FN LIST)
			(FN LIST)
		)

		; ---------------
		; Expects LIST to a a scheme list of items of 'right-type
		; Returns a set-union of the left-stars of all of tehse items.
		(define (left-star-union LIST)
			(delete-duplicates!
				(append-map!
					(lambda (item) (stars-obj 'left-stars ITEM))
					LIST)))

		; Same as above, but for the right
		(define (right-star-union LIST)
			(delete-duplicates!
				(append-map!
					(lambda (item) (stars-obj 'right-stars ITEM))
					LIST)))

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
