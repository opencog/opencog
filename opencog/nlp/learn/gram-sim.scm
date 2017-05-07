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
; right give the same answer.  Thus, an UnorderedLink is best for
; storing the pair. Specifically, it will be this:
;
;    SimilarityLink
;        WordNode "this"
;        WordNode "that"
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog persist) (opencog persist-sql))

(define cos-key (PredicateNode "*-Cosine Distance Key-*"))

; If the similarity is less than this, it is not saved.
(define cutoff 0.05)

; Define the atom at which the cosine similarity value will be stored.
(define (sim-pair WORD-A WORD-B) (SimilarityLink WORD-A WORD-B))

; Compute and store the similarity between the WORD, and the other
; words in the word-list
(define (batch-sim WORD WORD-LIST)

	(define (store-sim WORD-A WORD-B SIM)
		(store-atom
			(cog-set-value!
				(sim-pair WORD-A WORD-B) cos-key
				(FloatValue SIM (get-angle SIM)))))

	(for-each
		(lambda (wrd)
			(define sim (cset-vec-cosine WORD wrd))
			(if (< cutoff sim) (store-sim WORD wrd sim)))
		WORD-LIST)
)

; ---------------------------------------------------------------------
