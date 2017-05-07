;
; gram-sim.scm
;
; Batch-compute the grammatical similarity between all word-pairs.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute and store, as a batch process, the
; grammatical similarities between "all possible" pairs of words.
; It assumes that a large number of observations of pseudo-connector
; sets have already been made, and are currently stored in the database.
;
; Note that similarity scores are symmetric, so exchanging left and
; right give the same answer.

(define cos-key (PredicateNode "*-Cosine Distance Key-*))

; If the similarity is less than this, it is not saved.
(define cutoff 0.05)

; Compute and store the similarity between the WORD, and the other
; words in the word-list
(define (batch-sim WORD WORD-LIST)

	(define (store-sim WORD-A WORD-B SIM)
		(define simpr
			(EvaluationLink 
(cog-set-value!

	(for-each
		(lambda (wrd)
			(define sim (cset-vec-cosine (WORD wrd)))
			(if (< cutoff sim)
				(define dist (get-angle sim))
			))
		WORD-LIST)
)
