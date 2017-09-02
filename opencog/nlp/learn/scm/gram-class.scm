;
; gram-class.scm
;
; Merge words into grammatical categories.
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; When a pair of words are judged to be grammatically similar, they
; can be used to create a "grammatical class", containing both the
; words, and behaving as thier union/sum.  When a word is judged to
; belong to an existing grammatical-class, then some mechanism must
; be provided to add that word to the class.  This file implements
; the tools for creating and managing such classes.  It does not
; dictate how to judge when words belong to a class; this is done
; independently of the structure of the classes themselves.
;
; A grammatical class is represented as....
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
