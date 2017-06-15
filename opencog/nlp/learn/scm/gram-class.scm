;
; gram-class.scm
;
; Classify words into grammatical categories, using the "thresholding
; PCA" (Principal Component Analsysis) algo. 
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The overall algo is described in the language diary.  The bulk of the
; computation is performed in the (opencog matrix) toolkit.
;
; ---------------------------------------------------------------------

(use-modules (opencog) (opencog matrix))

(define (do-it)
	(let ((pca (make-pseudo-cset-api))
			(psa (add-pair-stars pca))
			(pfa (add-pair-freq-api psa))
		)

	(define pta (make-thresh-pca pfa))
	(define all-words (get-all-cset-words))



)
