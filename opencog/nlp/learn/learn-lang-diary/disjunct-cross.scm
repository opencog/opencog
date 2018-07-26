;
; disjunct-cross.scm
;
; Assorted ad-hoc collection of tools for understanding the
; word-similarity obtained via pseudo-disjunct overlap.
;
; Similar to disjunct-stats.scm, but includes stuff for cross-disjunct
; measures, and MI metrics.
;
; Used to create graphs for "Connetor Sets Distribution" 2018 version.
; These can ONLY be used by hand, by cutting and pasting the interesting
; bits from this file, into a guile prompt. This is more of a kind-of
; work-log of what was needed to generate those pictures, than anything
; else. Of course, this can be recycled for other datasets, too.
;
; Copyright (c) 2017,2018 Linas Vepstas
;

(use-modules (srfi srfi-1))

; ---------------------------------------------------------------------
; Load the dataset that is analyzed throughout.
(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog matrix))
(use-modules (opencog cogserver))
(start-cogserver)
(sql-open "postgres:///en_dj_two_sim?user=linas")

(define pca (make-pseudo-cset-api))
(define psa (add-pair-stars pca))
(define psc (add-pair-count-api psa))
(define psf (add-pair-freq-api psa))
(define psu (add-support-api psa))

(psa 'fetch-pairs)
(print-matrix-summary-report psa)

; -------------
; OK, so above provides filtered sim scores. That's old-school.
; Let's try it unfiltered. We have computes sims for the 640 most
; ferequently observed words ... make that word list
(define (get-sorted-basis LLOBJ)
	(define wldobj (add-pair-stars LLOBJ))
	(define basis (wldobj 'left-basis))
	(define supp-obj (add-support-api wldobj))
	(define (nobs ITEM) (supp-obj 'right-count ITEM))

	; Rank so that the commonest items are first in the list.
	(sort basis (lambda (ATOM-A ATOM-B) (> (nobs ATOM-A) (nobs ATOM-B))))
)
(define top-items (take (get-sorted-basis pca) 640))

; Create a list of similarity pairs.
(define (mksims items)
	(define (mkpr item rest lst)
		(if (null? rest) lst
			(mkpr (car rest) (cdr rest)
				(append lst (map (lambda (oit) (Similarity item oit)) rest)))))
	(mkpr (car items) (cdr items) '()))

(define good-sims (mksims top-items))
(length good-sims) ;; 204480 - as expected.

(cog-keys (car good-sims))

 (PredicateNode "*-SimKey Cross Cosine-*")
 (PredicateNode "*-SimKey pseudo-cset Cosine-*")
 (PredicateNode "*-SimKey Cross MI-*")
 (PredicateNode "*-SimKey pseudo-cset MI-*")

(define (sim-dj-cosine SIM)
	(define cos-key (PredicateNode "*-SimKey pseudo-cset Cosine-*"))
   (cog-value-ref (cog-value SIM cos-key) 0))

(define (sim-dj-mi SIM)
	(define mi-key (PredicateNode "*-SimKey pseudo-cset MI-*"))
   (cog-value-ref (cog-value SIM mi-key) 0))

(define ranked-dj-cos-sims
	(sort good-sims
		(lambda (a b) (> (sim-dj-cosine a) (sim-dj-cosine b)))))

(define ranked-dj-mi-sims
	(sort good-sims
		(lambda (a b) (> (sim-dj-mi a) (sim-dj-mi b)))))

(define scored-sims (score sim-dj-mi good-sims))
(define binned-sims (bin-count-simple scored-sims 300))

(let ((outport (open-file "/tmp/binned-sims.dat" "w")))
	(print-bincounts-tsv binned-sims outport)
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
