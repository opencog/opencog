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
		)

		(define (foo ITEM)
				)


	; Methods on this class.
	(lambda (message . args)
		(case message
			((foo)     (apply foo args))
			(else (apply llobj (cons message args))))
		)))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
