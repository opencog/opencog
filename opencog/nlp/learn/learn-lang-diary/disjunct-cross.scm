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
(define sorted-words (get-sorted-basis pca))

(define top-640-items (take sorted-words 640))
(define top-920-items (take sorted-words 920))

; Create a list of similarity pairs.
(define (mksims items)
	(define (mkpr item rest lst)
		(if (null? rest) lst
			(mkpr (car rest) (cdr rest)
				(append lst (map (lambda (oit) (Similarity item oit)) rest)))))
	(mkpr (car items) (cdr items) '()))

(define good-640-sims (mksims top-640-items))
(length good-640-sims) ;; 204480 - as expected.

(define good-920-sims (mksims top-920-items))
(length good-920-sims)  ;; 422740

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
   (define mi-kv (cog-value SIM mi-key))
   (define mi-val (if (null? mi-kv)
		-500 (cog-value-ref (cog-value SIM mi-key) 0)))
	; (if (null? mi-kv) (format #t "wtf ~A" SIM))
	; -inf MI is OK, it means zero overlap (cosine=0)
	;; (if (inf? mi-val) (format #t "wtf ~A" SIM))
	(if (inf? mi-val) (if (< 0 mi-val) 500 -500) mi-val))

(define scored-sims (score sim-dj-cosine good-640-sims))
(define scored-sims (score sim-dj-mi good-920-sims))
(define binned-sims (bin-count-simple scored-sims 300 -20 +10))

(let ((outport (open-file "/tmp/binned-sims.dat" "w")))
	(print-bincounts-tsv binned-sims outport)
	(close outport))

; ---------------------------------------------------------------------
; Non-binned, ranked plots...

(define ranked-dj-cos-sims
	(sort good-640-sims
		(lambda (a b) (> (sim-dj-cosine a) (sim-dj-cosine b)))))

(define ranked-dj-640-mi-sims
	(sort good-640-sims
		(lambda (a b) (> (sim-dj-mi a) (sim-dj-mi b)))))

(define ranked-dj-mi-sims
	(sort good-920-sims
		(lambda (a b) (> (sim-dj-mi a) (sim-dj-mi b)))))

(define (prt-sim sim port)
	(format port "~A  ~A  '~A .. ~A'\n"
		(sim-dj-cosine sim) (sim-dj-mi sim)
		(cog-name (gar sim)) (cog-name (gdr sim))))

(define (prt-mi-sim sim port)
	(format port "~A  '~A .. ~A'\n" (sim-dj-mi sim)
		(cog-name (gar sim)) (cog-name (gdr sim))))

; rank by cosine, print cosine and mi.
(let ((outport (open-file "/tmp/ranked-dj-cos-sims.dat" "w")))
	(define cnt 0) 
	(for-each (lambda (sim)
			(set! cnt (+ cnt 1))
			(format outport "~A  " cnt)
			(prt-sim sim outport))
		ranked-dj-cos-sims) 
	(close outport))

; rank by mi, print only mi
(let ((outport (open-file "/tmp/ranked-dj-mi-sims.dat" "w")))
	(define cnt 0) 
	(for-each (lambda (sim)
			(set! cnt (+ cnt 1))
			(format outport "~A  " cnt)
			(prt-mi-sim sim outport))
		ranked-dj-mi-sims) 
	(close outport))

; rank by mi, print cosine and mi
(let ((outport (open-file "/tmp/ranked-dj-640-mi-sims.dat" "w")))
	(define cnt 0) 
	(for-each (lambda (sim)
			(set! cnt (+ cnt 1))
			(format outport "~A  " cnt)
			(prt-sim sim outport))
		ranked-dj-640-mi-sims) 
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
