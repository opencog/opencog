;
; July 2018 version.
;
; Ad Hoc script to compute word-similarities in four different
; ways. These are:
;
; * cosines between words, using disjuncts as the vector-basis.
; * cosines between words, using cross-connectors as the vector basis.
; * symmetric MI, for each of the above two.
;
; The hypothesis is that the symmetric MI is superior. It seems to be
; a bit slower to compute. It should be additive, in the way that the
; cosines are not.
;
; Just for the hell of it, we'll do the top 2K words, so that's
; 4M additional atoms. A lot but not overwhelming.
;
; So, here we go:
; ALTER DATABASE en_dj_ptwo RENAME TO en_dj_two_sim;
;

; (sql-open "postgres:///en_dj_two_sim?user=linas&password=asdf")

; -------------
; Cosines between words, using dj's
(define pca (make-pseudo-cset-api))
(define psa (add-pair-stars pca))
(define pta (add-transpose-api psa))

; (load-atoms-of-type 'SimilarityLink)
; (cog-count-atoms 'SimilarityLink)
; (pca 'fetch-pairs)

(define pco (add-pair-cosine-compute pta))
(define bco
	(batch-similarity pta #f "pseudo-cset Cosine-*" 0.0
		(lambda (wa wb) (pco 'right-cosine wa wb))))

; (pco 'right-cosine (Word "other") (Word "same")) ; 0.5866011982435116
; (bco 'compute-similarity (Word "other") (Word "same"))

; (bco 'batch-compute 12)
; Done 10/12 frac=100.0% Time: 2201 Done: 98.5% rate=0.030 prs/sec
; Done 70/72 frac=97.42% Time: 23066 Done: 100.0% rate=0.108 prs/sec

; -------------
(define minus-inf (- 0 (inf)))

; FMI between words, using dj's -- below works
(define pcam (make-pseudo-cset-api))
(define psam (add-pair-stars pcam))
(define ptam (add-transpose-api psam))
(define pmi (add-symmetric-mi-compute psam))
(define bmi
	(batch-similarity ptam #f "pseudo-cset MI-*" minus-inf
		(lambda (wa wb) (pmi 'mmt-fmi wa wb))))

; (pmi 'mmt-fmi (Word "other") (Word "same")) ; 4.123194356470049
; (bmi 'compute-similarity (Word "other") (Word "same"))

; (bmi 'batch-compute 12)
; Done 10/12 frac=100.0% Time: 1299 Done: 98.5% rate=0.050 prs/sec
; Done 70/72 frac=97.42% Time: 13926 Done: 100.0% rate=0.179 prs/sec

(map (lambda (n) (bmi 'batch-compute n)) (iota 10 40 25))


; ============================================
; Cosines between words, using crosses
(define cra (make-shape-vec-api))
(define crs (add-pair-stars cra))
(define crt (add-transpose-api crs))

; (cra 'fetch-pairs)

(define cco (add-pair-cosine-compute crt))
(define bcr
	(batch-similarity crt #f "Cross Cosine-*" 0.0
		(lambda (wa wb) (cco 'right-cosine wa wb))))

; (cco 'right-cosine (Word "other") (Word "same")) ; 0.5853349406547999
; (bcr 'compute-similarity (Word "other") (Word "same"))

; (bcr 'batch-compute 12)
; Done 10/14 Frac=11.76% Time: 4591 Done: 93.4% Rate=0.002 prs/sec (459.1 sec/pr)

; -------------
; FMI between words, using crossovers -- below works
(define cram (make-shape-vec-api))
(define crsm (add-pair-stars cram))
(define crtm (add-transpose-api crsm))

(define cmi (add-symmetric-mi-compute crsm))
(define mib
	(batch-similarity crtm #f "Shape MI-*" minus-inf
		(lambda (wa wb) (cmi 'mmt-fmi wa wb))))

; (cmi 'mmt-fmi (Word "other") (Word "same")) ; 3.2194667964612314
; (mib 'compute-similarity (Word "other") (Word "same")) ;

; (mib 'batch-compute 12)
; Done 10/12 frac=95.38% Time: 20314 Done: 98.5% rate=-0.00 prs/sec

(map (lambda (n) (mib 'batch-compute n)) (iota 4 500 2))

; -------------
(cog-count-atoms 'SimilarityLink)

(define (store-sims)
	(cog-map-type (lambda (atm) (store-atom atm) #f) 'SimilarityLink))

(define (store-regularly)
	(sleep 1200)
;	(load-atoms-of-type 'SimilarityLink)
	(sleep 1200)
;	(load-atoms-of-type 'SimilarityLink)
	(sleep 1200)
	(store-sims)
	(format #t "Done storing ~A ~A\n"
		(cog-count-atoms 'SimilarityLink)
		(strftime "%c" (localtime (current-time))))
	(load-atoms-of-type 'SimilarityLink)
	(store-regularly))
