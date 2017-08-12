;
; disjunct-stats.scm
;
; Assorted ad-hoc collection of tools for understanding the
; word-simillarity obtained via pseudo-disjunct overlap.
;
; These were used to create the section "Connetor Sets 7 May 2017"
; in the diary. These can ONLY be used by hand, by cutting and pasting
; the interesting bits from this file, into a guile prompt. This is
; more of a kind-of work-log of what was needed to generate those
; pictures, than anything else. Of course, this can be recycled for
; other datasets, too.
;
; Copyright (c) 2017 Linas Vepstas
;

(use-modules (srfi srfi-1))

; ---------------------------------------------------------------------
; Load the dataset that is analyzed throughout.
(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog matrix))
(use-modules (opencog cogserver))
(start-cogserver)
; (sql-open "postgres:///en_pairs_sim?user=linas")
(sql-open "postgres:///en_pairs_rfive_mtwo?user=linas")

(define pca (make-pseudo-cset-api))
(define psa (add-pair-stars pca))
(define psc (add-pair-count-api psa))
(define psf (add-pair-freq-api psa))
(define psu (add-support-api psa))

(psa 'fetch-pairs)                  ; 4436 secs
(print-matrix-summary-report psa)


; (fetch-all-words)
; (length (get-all-words))  ; 

(define ac (psa 'left-basis))
(length ac)               ; 137078

(define ad (psa 'right-basis))
(length ad)               ; 6239997

;;; (fetch-all-sims)


; ---------------------------------------------------------------------
; Ranking and printing utilities
;
; Assign each word a score, using SCORE-FN
(define (score SCORE-FN WORD-LIST)
	(map (lambda (wrd) (cons (SCORE-FN wrd) wrd)) WORD-LIST))

; Assign each word a score, using SCORE-FN, and then rank them by
; score: i.e. sort them, with highest score first.
(define (score-and-rank SCORE-FN WORD-LIST)
	(sort
		(map (lambda (wrd) (cons (SCORE-FN wrd) wrd)) WORD-LIST)
		(lambda (a b) (> (car a) (car b)))))

; Print to port a tab-separated table of rankings
(define (print-ts-rank-fn scrs port fn-str)
	(define cnt 0)
	(for-each
		(lambda (pr)
			(set! cnt (+ cnt 1))
			(format port "~A	~A	\"~A\"\n" cnt (car pr) (fn-str (cdr pr))))
		scrs))

(define (print-ts-rank scrs port)
	(print-ts-rank-fn scrs port cog-name))

(define (print-ts-rank-cset scrs port)
   (print-ts-rank-fn scrs port
		(lambda (x) (cog-name (gar x)))))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; A list of all words that have csets. (Not all of the words
; in the database got tagged with a cset)
(define all-cset-words (psa 'left-basis))  ; (get-all-cset-words)

; A list of all disjucnts (appearing in all csets)
(define all-disjuncts (psa 'right-basis)) ; (get-all-disjuncts))

(define all-csets (get-all-csets))

; Words with 100 or more observations. rfive_mtwo has 9425 of these.
; There are 25505 with 20 or more
; There are 20284 with 30 or more
; There are 3263 with 400 or more
(define top-cset-words
	(filter (lambda (wrd) (<= 100 (cset-vec-word-observations wrd)))
		all-cset-words))

; ---------------------------------------------------------------------
; Print the support, size and length of for a word,

(define (show-counts word-str)
	(define w (WordNode word-str))
	(format #t "Suport=~A Count=~A Length=~A\n"
		(psu 'right-support w)
		(psu 'right-count w)
		(psu 'right-length w)))

; ---------------------------------------------------------------------
; A sorted list of score-word pairs, where the score is the count
; of the cset observations. Note that this score is *identical* to the
; number of times that the word was observed during MST parsing. That is
; because exactly one disjunct is extracted per word, per MST parse.
(define sorted-word-obs
	(score-and-rank cset-vec-word-observations all-cset-words))

; Print above to a file, so that it can be graphed.
(let ((outport (open-file "/tmp/ranked-word-obs.dat" "w")))
	(print-ts-rank sorted-word-obs outport)
	(close outport))

; A sorted list of score-disjunct pairs, where the score is the count
; of the cset observations.
(define sorted-dj-obs
	(score-and-rank cset-vec-dj-observations all-disjuncts))

; Print above to a file, so that it can be graphed.
(let ((outport (open-file "/tmp/ranked-dj-obs.dat" "w")))
	(print-ts-rank-fn sorted-dj-obs outport get-disjunct-string)
	(close outport))

; A sorted list of score-cset pairs, where the score is the count
; of the cset observations.
(define sorted-cset-obs (score-and-rank get-count all-csets))

; Print above to a file, so that it can be graphed.
(let ((outport (open-file "/tmp/ranked-cset-obs.dat" "w")))
	(print-ts-rank-fn sorted-cset-obs outport get-cset-string)
	(close outport))

; ---------------------------------------------------------------------
; Compute the average number of observations per disjunct.
(define (avg-obs WORD)
	(/ (cset-vec-word-observations WORD) (cset-vec-word-support WORD)))

; Compute the average number of observations per disjunct.
; Discard anything with less than 100 observations.
(define top-sorted-avg
	(score-and-rank avg-obs top-cset-words))

(let ((outport (open-file "/tmp/ranked-avg.dat" "w")))
	(print-ts-rank top-sorted-avg outport)
	(close outport))

; --------------------------
; As above, but this time, instead of ranking, we bin-count.
;
(define sorted-avg
	(score-and-rank avg-obs all-cset-words))

(define sorted-avg
	(score-and-rank avg-obs top-cset-words))

(define binned-avg (bin-count-simple sorted-avg 200 1.0 20.0))

(let ((outport (open-file "/tmp/binned-avg.dat" "w")))
	(print-bincounts-tsv binned-avg outport)
	(close outport))

; ---------------------------------------------------------------------
; A sorted list of the support of a word.
; The support is simply how many basis elements of a vector are
; non-zero.  Equivalently, it is the size of the set of unique
; disjuncts associated with a word (counted without multiplicity).

(define sorted-support (score-and-rank cset-vec-word-support all-cset-words))

(let ((outport (open-file "/tmp/ranked-support.dat" "w")))
	(print-ts-rank sorted-support outport)
	(close outport))

; ---------------------------------------------------------------------
; A sorted list of score-word pairs, where the score is the cset length

(define sorted-lengths (score-and-rank cset-vec-word-len all-cset-words))

(let ((outport (open-file "/tmp/ranked-lengths.dat" "w")))
	(print-ts-rank sorted-lengths outport)
	(close outport))

; ---------------------------------------------------------------------
; Consider, for example, the length-squared, divided by the number
; of observations.
(define (lensq-vs-obs wrd)
	(define len (cset-vec-word-len wrd))
	(/ (* len len) (cset-vec-word-observations wrd)))

; The length vs observation ranking; but discard everything
; with a small number of observations.
(define sorted-lensq-norm-top
	(score-and-rank lensq-vs-obs top-cset-words))

(let ((outport (open-file "/tmp/ranked-sqlen-norm.dat" "w")))
	(print-ts-rank sorted-lensq-norm-top outport)
	(close outport))

(define sorted-lensq-norm
	(score-and-rank lensq-vs-obs all-cset-words))

(define binned-sqlen-norm (bin-count-simple sorted-lensq-norm 200 0.0 100.0))

(define binned-sqlen-norm (bin-count sorted-lensq-norm 100
	(lambda (item) (/ (log (first item)) (log 2.0)))
	(lambda (item) 1)
	1.0 10.0))

(let ((outport (open-file "/tmp/binned-sqlen-norm.dat" "w")))
	(print-bincounts-tsv binned-sqlen-norm outport)
	(close outport))

; ---------------------------------------------------------------------
; RMS deviation from mean.
;  sum_i (x-a)^2 = sum_i x^2 - 2a sum_i x + a^2 sum_i
; but now divide by N = sum_i, again, to get
;  sum_i (x-a)^2 = <x^2> - a^2
;
(define (avg-obs WORD)
	(/ (cset-vec-word-observations WORD) (cset-vec-word-support WORD)))

(define (meansq-obs WORD)
	(define len (cset-vec-word-len WORD))
	(/ (* len len) (cset-vec-word-support WORD)))

(define (rms-deviation WORD)
	(define avg (avg-obs WORD))
	(sqrt (- (meansq-obs WORD) (* avg avg))))

; Compute the RMS deviation from the average number of observations
; per disjunct.
; Discard anything with less than 100 observations.
(define sorted-rms
	(score-and-rank rms-deviation top-cset-words))

(let ((outport (open-file "/tmp/ranked-rms.dat" "w")))
	(print-ts-rank sorted-rms outport)
	(close outport))

;
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Vertex degrees and hubiness.
; The vertex degree is the same thing as the number of connectors
; on a disjunct.
; Hubiness is the second moment of the vertex degree.

(define (avg-con-count wrd)
	(/ (cset-vec-connectors wrd) (cset-vec-word-observations wrd)))

; Rank by average disjunct size; but discard everything
; with a small number of observations.
(define sorted-avg-connectors
	(score-and-rank avg-con-count top-cset-words))

(define sorted-avg-connectors
	(score-and-rank avg-con-count all-cset-words))

(let ((outport (open-file "/tmp/ranked-avg-connectors.dat" "w")))
	(print-ts-rank sorted-avg-connectors outport)
	(close outport))

; ----
; Second moment is sum (c^2/n) - (sum c/n)^2
; RMS is sqrt (sum (c^2/n) - (sum c/n)^2)
(define (moment-con-count wrd)
	(define meansq
		(/ (cset-vec-lp-connectors wrd 2) (cset-vec-word-observations wrd)))
	(define avg (avg-con-count wrd))
	(sqrt (- meansq (* avg avg))))

(define sorted-hub-connectors
	(score-and-rank moment-con-count top-cset-words))

(let ((outport (open-file "/tmp/ranked-hub-connectors.dat" "w")))
	(print-ts-rank sorted-hub-connectors outport)
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
;
; Distributions for two particular words.

(define dj-prince (score-and-rank get-count
		(get-cset-vec (Word "Prince"))))

(define dj-think (score-and-rank get-count
		(get-cset-vec (Word "think"))))

(define dj-long (score-and-rank get-count
		(get-cset-vec (Word "long"))))

(define dj-fact (score-and-rank get-count
		(get-cset-vec (Word "fact"))))

(define dj-from (score-and-rank get-count
		(get-cset-vec (Word "from"))))

; Print to port a tab-separated table of rankings
(define (print-ts-dj-rank cset-a cset-b 
                cset-c cset-d cset-e port)
	(define idx 0)
	(for-each
		(lambda (pra prb prc  prd pre)
			(set! idx (+ idx 1))
			(format port "~A	~A	~A	~A	~A	~A\n" idx 
				(car pra) (car prb)
				(car prc) (car prd)
				(car pre)))
		cset-a cset-b cset-c cset-e cset-e ))

(let ((outport (open-file "/tmp/ranked-dj-prince.dat" "w")))
	(print-ts-dj-rank 
		dj-prince dj-think dj-long dj-fact dj-from outport)
	(close outport))

; ---------------------------------------------------------------------
; Sum over distributions. Basically, above gave five ranking
; distributions, one for each word.  They can be averaged together
; to get a smoother graph.  Here, we create a ranking for each
; word, and then average them all together. This is kind of hokey,
; in the end, but whatever.

; Create and zero out array. 312K is just some large "big-enough"
; number big enough to hold data without overflowing.
(define dj-sum (make-array 0 312500))

; Create a ranked list of disjuncts for WORD
(define (make-ranked-dj-list WORD)
	(score-and-rank get-count (get-cset-vec WORD)))

; Accumulate the disjunct counts for WORD, into dj-sum
(define (accum-dj-counts WORD)
	(define idx 0)
	(for-each
		(lambda (pr)
			(array-set! dj-sum (+ (car pr) (array-ref dj-sum idx)) idx)
			(set! idx (+ idx 1)))
		(make-ranked-dj-list WORD)))

; Accumulate the disjunct counts for all the words in the word-list
(define (accum-dj-all WORD-LIST)
	(for-each
		(lambda (word) (accum-dj-counts word))
		WORD-LIST))

; Accumulate all, but only for those with 100 or more observations.
(accum-dj-all top-cset-words)

(define (print-dj-acc port)
	(define idx 0)
	(for-each
		(lambda (cnt)
			(set! idx (+ idx 1))
			(if (< 0 cnt)
				(format port "~A	~A\n" idx cnt)))
		(array->list dj-sum)))

(let ((outport (open-file "/tmp/ranked-dj-counts.dat" "w")))
	(print-dj-acc outport)
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Entropies.

; Several ways of counting the same thing. These should all
; give the same result.

(define total-cset-count (get-total-cset-count))

; Get the "fractional entropy".
(define (fractional-entropy FUNC LIST)
	(fold
		(lambda (item sum)
			(define pitem (/ (FUNC item) total-cset-count))
			(- sum (* pitem (log pitem))))
		0
		LIST))

; Get the "word entropy".
(define word-entropy
	(fractional-entropy cset-vec-word-observations all-cset-words))
(define word-entropy-bits (/ word-entropy (log 2.0)))

(define disjunct-entropy
	(fractional-entropy cset-vec-dj-observations all-disjuncts))
(define disjunct-entropy-bits (/ disjunct-entropy (log 2.0)))

(define cset-entropy-bits
	(fold
		(lambda (word sum) (+ sum  (cset-vec-word-entropy word)))
		0
		all-cset-words))

; A sloppy, slow, wasteful algorithm
(define (get-total-mi-slow)
	(define prog 0)
	(define prog-len (length all-csets))
	(fold
		(lambda (cset sum)
			(define word (cset-get-word cset))
			(define dj (cset-get-disjunct cset))
			(define p-word (cset-vec-frequency word))
			(define p-dj (cset-vec-frequency dj))
			(define p-cset (get-cset-frequency cset))

			(set! prog (+ prog 1))
			(if (eqv? 0 (modulo prog 100))
				(format #t "doing ~A of ~A\n" prog prog-len))
			(+ sum (* p-cset (log (/ p-cset (* p-word p-dj)))))
		)
		0
		all-csets))

(define slow-total-mi (get-total-mi-slow))

; A much faster, optimized version. Still takes tens of minutes
; to run.
(define (get-total-mi-bits)
	(define prog 0)
	(define prog-len (length all-cset-words))
	(define sum 0)
	(for-each
		(lambda (word)
			(set! prog (+ prog 1))
			(if (eqv? 0 (modulo prog 100))
				(format #t "doing ~A of ~A\n" prog prog-len))
			(set! sum (+ sum (cset-vec-word-mi word))))
		all-cset-words)
	sum
)

(define total-mi-bits (get-total-mi-bits))

; The in-a-jiffy version which needs no new computations:
(define total-mi-bits
	(- (word-entropy-bits disjunct-entropy-bits) cset-entropy-bits)

; ---------------------------------------------------------------------
; A simple graph of how many words were observed once, twice, etc.
; So: first column: how many times a word was observed.
; Second column: the number of words that were observed tat many times.
;
(define sorted-word-counts
	(score-and-rank cset-vec-word-observations all-cset-words))
(define binned-word-counts
	(bin-count-simple sorted-word-counts 400 0.5 400.5))

(let ((outport (open-file "/tmp/binned-word-counts.dat" "w")))
	(print-bincounts-tsv binned-word-counts outport)
	(close outport))

;--------------
; Create a binned graph of the number of words observed with a given
; log-liklihood.  Although this is in the binning tradition of the
; other binned graphs, its really the same thing as the log-log
; graph of the word-counts, but renormalized.
; (define pca (make-pseudo-cset-api))
; (define psa (add-pair-stars pca))
; (define pcw (add-pair-wildcards psa))
; (define pmi (add-pair-mi-compute psa))
(define (cset-vec-word-logli WORD)
	(psf 'right-wild-logli WORD))

(define sorted-word-logli (score-and-rank cset-vec-word-logli all-cset-words))
(define binned-word-logli (bin-count-simple sorted-word-logli 1200 9.0 24.0))

(let ((outport (open-file "/tmp/binned-word-logli.dat" "w")))
	(print-bincounts-tsv binned-word-logli outport)
	(close outport))

; -------
; Rank words according to thier fractional entropy
(define (cset-vec-word-ent WORD) (psf 'right-wild-fentropy WORD))

; rank only the top-100
(define sorted-word-ent (score-and-rank cset-vec-word-ent top-cset-words))

; Print above to a file, so that it can be graphed.
(let ((outport (open-file "/tmp/ranked-word-ent.dat" "w")))
	(print-ts-rank sorted-word-ent outport)
	(close outport))

(define sorted-word-ent (score-and-rank cset-vec-word-ent all-cset-words))
(define binned-word-ent (bin-count-simple sorted-word-ent 100))

(let ((outport (open-file "/tmp/binned-word-ent.dat" "w")))
	(print-bincounts-tsv binned-word-ent outport)
	(close outport))

;--------------
; cheat method, assumes we are not making errors,
; it would be better to double-check this by running the
; sums the other way.
(define (cset-vec-word-crazy WORD)
	(- (pmi 'compute-right-mi WORD)
		(+ (pmi 'right-wild-logli WORD)
			(pmi 'compute-right-fentropy WORD))))

(define sorted-word-crazy (score-and-rank cset-vec-word-crazy all-cset-words))
(define binned-word-crazy (bin-count-simple sorted-word-crazy 100))

(let ((outport (open-file "/tmp/binned-word-crazy.dat" "w")))
	(print-bincounts-tsv binned-word-crazy outport)
	(close outport))

(define (cset-vec-word-crazy2 WORD)
	(- (pmi 'compute-right-mi WORD)
		(pmi 'compute-right-fentropy WORD)))

(define sorted-word-crazy2 (score-and-rank cset-vec-word-crazy2 all-cset-words))
(define binned-word-crazy2 (bin-count-simple sorted-word-crazy2 100))

(let ((outport (open-file "/tmp/binned-word-crazy2.dat" "w")))
	(print-bincounts-tsv binned-word-crazy2 outport)
	(close outport))

; ---------------------------------------------------------------------
; Rank word-disjunct pairs according to thier fractonal MI.

(define (cset-mi CSET) (psf 'pair-fmi CSET))

(define sorted-cset-mi (score-and-rank cset-mi all-csets))
(define scored-cset-mi (score cset-mi all-csets))

; Print above to a file, so that it can be graphed.
(let ((outport (open-file "/tmp/ranked-cset-mi.dat" "w")))
	(print-ts-rank-cset sorted-cset-mi outport)
	(close outport))

(define binned-cset-mi (bin-count-simple sorted-cset-mi 100))
(define binned-cset-mi (bin-count-simple scored-cset-mi 200))

(let ((outport (open-file "/tmp/binned-cset-mi.dat" "w")))
	(print-bincounts-tsv binned-cset-mi outport)
	(close outport))

(define (cset-freq CSET) (psf 'pair-freq CSET))

(define weighted-cset-mi
	(bin-count-weighted scored-cset-mi 200
		(lambda (scored-item) (cset-freq (cdr scored-item)))))

(let ((outport (open-file "/tmp/weighted-cset-mi.dat" "w")))
	(print-bincounts-tsv weighted-cset-mi outport)
	(close outport))

(define (cset-logli CSET) (pfrq 'pair-logli CSET))

(define wlogli-cset-mi
	(bin-count-weighted sorted-cset-mi 200
		(lambda (scored-item) (cset-logli (cdr scored-item)))))

(let ((outport (open-file "/tmp/wlogli-cset-mi.dat" "w")))
	(print-bincounts-tsv wlogli-cset-mi outport)
	(close outport))

; ---------------------------------------------------------------------

(define (cset-vec-word-mi WORD) (psf 'right-wild-fmi WORD))

; Rank words according to the marginal MI between them and thier disjuncts
(define sorted-word-mi (score-and-rank cset-vec-word-mi all-cset-words))

; Like above, but high-ferequency only
(define sorted-word-mi-hi-p
	(score-and-rank cset-vec-word-mi top-cset-words))

; Print above to a file, so that it can be graphed.
(let ((outport (open-file "/tmp/ranked-word-mi.dat" "w")))
	(print-ts-rank sorted-word-mi outport)
	(close outport))

(let ((outport (open-file "/tmp/ranked-word-mi-hi-p.dat" "w")))
	(print-ts-rank sorted-word-mi-hi-p outport)
	(close outport))

; ---------------------------------------------------------------------
; Create the binned distribution graphs for word-MI

(define binned-word-mi (bin-count-simple sorted-word-mi 100))

(define (new-cset-vec-word-mi WORD)
	(define pca (make-pseudo-cset-api))
	(define pmi (add-pair-mi-compute pca))
	(pmi 'compute-right-fmi WORD)
)

(define osorted-word-mi (score-and-rank new-cset-vec-word-mi all-cset-words))

(let ((outport (open-file "/tmp/binned-word-mi.dat" "w")))
	(print-bincounts-tsv binned-word-mi outport)
	(close outport))

; ---------------------------------------------------------------------
; Create the binned distribution graphs for dj-MI

(define (cset-vec-disjunct-mi DJ) (psf 'left-wild-fmi DJ))

(define sorted-dj-mi (score-and-rank cset-vec-disjunct-mi all-disjuncts))
(define scored-dj-mi (score cset-vec-disjunct-mi all-disjuncts))

(define binned-dj-mi (bin-count-simple scored-dj-mi 200))

(let ((outport (open-file "/tmp/binned-dj-mi.dat" "w")))
	(print-bincounts-tsv binned-dj-mi outport)
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; disjunct tools.

(filter (lambda (cset) (< 0 (get-count cset))) (cog-incoming-by-type
(Word "horses") 'Section))

; ---------------------------------------------------------------------

(define psu (add-support-api psa))
(define long-words
	(filter (lambda (word) (<= 256 (psu 'right-length word)))
		(psu 'left-basis)))

(length long-words) ; 6568 of len >= 16
; and 32 <= len has 3278 
; 64 <= len has 1608
; 130 <= has 783
; 128 <= has 797 .. lets do those.
; 256  has 427
; 512  has 245
; 1024 has 130
; 2048 has 69
; 4096 has 37...
; 4500 has 32

(define psm (add-similarity-api psa #f))
(psm 'fetch-pairs)

(define pslon
   (add-generic-filter psa
      (lambda (word) (<= 16 (psu 'right-length word)))
      (lambda (dj) #t)
      (lambda (dj) #t)
      (lambda (dj) #t)
      (lambda (dj) #t)
      "cut len<=16"
      #f))

(define psls (add-pair-stars pslon))
(define pss (batch-similarity psls #f #f 0.1))
(pss 'batch-compute)
(pss 'paralel-batch 3)

(cog-map-type ato 'SimilarityLink)

(define ranked-long
	(sort long-words
		(lambda (a b) (> (psu 'right-count a) (psu 'right-count b)))))
		; (lambda (a b) (> (psu 'right-support a) (psu 'right-support b)))))
		; (lambda (a b) (> (psu 'right-length a) (psu 'right-length b)))))

; ---------------------------------------------------------------------
; Similarity.  Cosine distance.
; Assumes that cosine similarities have been batch-computed, already,
; using the code in gram-sim.scm

(define cos-key (PredicateNode "*-Cosine Sim Key-*"))

(define all-sims '())
(cog-map-type
	(lambda (ato) 
		(define val (cog-value ato cos-key)) 
		(if (not (null? val))
			(set! all-sims (cons ato all-sims)))
		 #f)
	'SimilarityLink)

(length all-sims)    ;  317206

(define (sim-cosine SIM)
	(define cos-key (PredicateNode "*-Cosine Sim Key-*"))
   (cog-value-ref (cog-value SIM cos-key) 0))

(define (filter-sim LEN)
	(filter
		(lambda (sim) (and
				; (< 0.5 (sim-cosine sim))
				(< LEN (cset-vec-word-len (gar sim)))
				(< LEN (cset-vec-word-len (gdr sim)))))
		all-sims))

(define good-sims (filter-sim 256))
(length good-sims)   ;  

(define ranked-sims
	(sort good-sims
		(lambda (a b) (> (sim-cosine a) (sim-cosine b)))))

(define (prt-sim sim port)
	(format port "~A	'~A .. ~A'\n" (sim-cosine sim)
		(cog-name (gar sim)) (cog-name (gdr sim))))

(let ((outport (open-file "/tmp/ranked-sims-256.dat" "w")))
	(define cnt 0)
	(for-each (lambda (sim)
			(set! cnt (+ cnt 1))
			(format outport "~A	" cnt)
			(prt-sim sim outport))
		ranked-sims)
	(close outport))

; ------ again, but binned.

(define scored-sims (score sim-cosine all-sims)) 

(define binned-sims (bin-count-simple scored-sims 300))

(let ((outport (open-file "/tmp/binned-sims.dat" "w")))
	(print-bincounts-tsv binned-sims outport)
	(close outport))

(define all-good-sims
	(filter
		(lambda (sim) (and
				(< 4 (cset-vec-word-len (gar sim)))
				(< 4 (cset-vec-word-len (gdr sim)))))
		all-sims))

(length all-good-sims) ; 2172114 = 2M

(define scored-good-sims (score sim-cosine all-good-sims))
(define binned-good-sims (bin-count-simple scored-good-sims 100))

(let ((outport (open-file "/tmp/binned-good-sims.dat" "w")))
	(print-bincounts-tsv binned-good-sims outport)
	(close outport))

(define all-good-words (filter (lambda (w) (< 4 (cset-vec-word-len w))) ac))

(length all-good-words)   ;  5544 of length 4 or more

; ---------------------------------------------------------------------
; Connector-printing utilities.
(define (print-connector CON)
	(string-append (cog-name (gar CON)) (cog-name (gdr CON)) " "))

(define (section->dj-str SEC)
	(string-concatenate
		(map print-connector (cog-outgoing-set (gdr SEC))))
)

(define (print-disjuncts WORD CUT)
	(define cnt 0)
	(define all-secs (cog-incoming-by-type WORD 'Section))
	(define sorted-secs (sort all-secs
		 (lambda (a b) (> (get-count a) (get-count b)))))
	(define (prt-one con)
		(set! cnt (+ 1 cnt))
		(if (<= CUT (get-count con))
			(format #t "~D  ~A\n" (get-count con) (section->dj-str con))))
	(for-each prt-one sorted-secs)
	(format #t "Total of ~D disjuncts\n" cnt))

; ---------------------------------------------------------------------

(define wpa (make-any-link-api))
(define wps (add-pair-stars wpa))
(define wpf (add-pair-freq-api wps)

(fetch-incoming-set (List (Word "the") (Word "city"))
 (wpf 'pair-fmi (List (Word "the") (Word "city")))

; ---------------------------------------------------------------------
; Scatterplot stuff. Grep for drank-helper in the notes file. Its
; reproduced here, updated and modernized.

; Create a sorted list of words, such that the next word in the list
; has the highest cosine of all to the previous word in the list.
; wlist -- list of words
; drl -- set this to nil.
(define (drank-helper wlist drl)
	(define (get-cos A B)
		(sim-cosine (SimilarityLink A B)))
	(define rest (cdr wlist))
	; (format #t "duude enter wlsi=~A\n	and drl=~A\n" (length wlist) drl)
	; If the list is one item long, we are done.
	(if (null? rest) (cons (car wlist) drl)
		; Otherwise, find the word with the highest cosine to the previous
		; word that was picked.  rankw is the previous one picked.
		; next-rankw will be th next one.
		(let ((rankw (car drl))
				(next-rankw '()))
			(fold
				(lambda (item biggest)
					(define cosi (get-cos rankw item))
					(if (and (> cosi biggest) (not (equal? item rankw)))
						(begin (set! next-rankw item) cosi) biggest))
				-1e20
				wlist)
			(drank-helper
				(remove (lambda (w) (equal? w next-rankw)) wlist)
				(cons next-rankw drl)))))

(define (drank wlist)
	(drank-helper (cdr wlist) (cons (car wlist) '())))

(define dranked-long (drank (take ranked-long 40)))
(define dranked-long (drank (take ranked-long 300)))
(define dranked-long (drank ranked-long))

(define eranked-long (reverse dranked-long))

(write-flo "/tmp/scat-dcos.flo" eranked-long pair-cos)
(write-flo "/tmp/scat-ecos.flo" eranked-long pair-cos)

(define (prt-sim LST)
	(if (not (null? (cdr LST))) (begin
		(format #t "~A .. ~A  ~5F\n" (car LST) (cadr LST)
			(get-cos (Word (car LST)) (Word (cadr LST))))
		(prt-sim (cdr LST)))))



; ---------------------------------------------------------------------
; what fraction of the rows have more than N observations?
; (subtotaled for that entire row?)

(define (cnt-obs-rows n lst)
	(if (<= 0 n) 
		(let* ((fsa (add-subtotal-filter psa 0 n 0))
				(bs (fsa 'left-basis-size)))
			(format #t "N=~d  sz=~d\n" n bs)
			(cnt-obs-rows (- n 2) (cons (cons n bs) lst)))
		lst))

(define row-dist (cnt-obs-rows 100 '()))

(define (print-tsv-count scrs port max)
	(define cnt 0)
	(for-each
		(lambda (pr)
			(set! cnt (+ cnt 1))
			(format port "~A	~A	~A	~A\n" cnt (car pr) (cdr pr) (/ (cdr pr max)))
		scrs))

(let ((outport (open-file "/tmp/count-rows.dat" "w")))
	(print-tsv-count row-dist outport)
	(close outport))

(define (cnt-obs-cols n lst)
	(if (<= 0 n) 
		(let* ((fsa (add-subtotal-filter psa n 0 0))
				(bs (fsa 'right-basis-size)))
			(format #t "N=~d  sz=~d\n" n bs)
			(cnt-obs-cols (- n 5) (cons (cons n bs) lst)))
		lst))

(define col-dist (cnt-obs-cols 100 '()))

; ---------------------------------------------------------------------
; Support -- This is for the table about filters.

(define fsa (add-subtotal-filter psa 1 1 0))
(define fsb (add-subtotal-filter psa 4 4 0))
(define fsc (add-subtotal-filter psa 10 10 0))
(define fsd (add-subtotal-filter psa 30 30 0))

(define ssa (add-support-compute fsa))
(define ssb (add-support-compute fsb))
(define ssc (add-support-compute fsc))
(define ssd (add-support-compute fsd))

(ssa 'total-support)  ; 4096008
(ssa 'total-count)    ; 12039800.0
(ssb 'total-support)  ; 2807987
(ssb 'total-count)    ; 9508010.0
(ssc 'total-support)  ; 2423739
(ssc 'total-count)    ; 8733613.0
(ssd 'total-support)  ; 1976731
(ssd 'total-count)    ; 8046133


(define fse (add-subtotal-filter psa 30 10 0))
(define fsf (add-subtotal-filter psa 30 10 4))

(define fsg (add-subtotal-filter psa 50 30 0))
(define fsh (add-subtotal-filter psa 50 30 4))
(define fsi (add-subtotal-filter psa 50 30 10))

(define sse (add-support-compute fse))
(define ssf (add-support-compute fsf))
(define ssg (add-support-compute fsg))
(define ssh (add-support-compute fsh))
(define ssi (add-support-compute fsi))

(sse 'total-support)  ; 2261327
(sse 'total-count)    ; 8478382.0
(ssf 'total-support)  ; 269459
(ssf 'total-count)    ; 5524982.0
(ssg 'total-support)  ; 1875670
(ssg 'total-count)    ; 7863145.0
(ssh 'total-support)  ; 255698
(ssh 'total-count)    ; 5405406.0
(ssi 'total-support)  ; 100542
(ssi 'total-count)    ; 4363640.0

; ---------------------------------------------------------------------
; Redo the PCA results, but with a different filtering

; This cut everything except for 797 words. Cuts no disjuncts.
;
(define ps-128
   (add-generic-filter psa
      (lambda (word) (<= 128 (psu 'right-length word)))
      (lambda (dj) #t)
      (lambda (dj) #t)
      (lambda (dj) #t)
      (lambda (dj) #t)
      "cut len<=128"
      #f))

(define long-words
	(filter (lambda (word) (<= 128 (psu 'right-length word)))
		(psu 'left-basis)))

; Its OK to use make-cosine-matrix -- this will still give the correct
; cosines for the above, because the above does Not cut any disjuncts.
(define pci (make-cosine-matrix ps-128))
(define pti (make-power-iter-pca pci 'left-unit))

(define feig (pti 'make-left-unit long-words))
(define lit (pti 'left-iterate feig 8)) 
(pti 'left-print lit 20) 

 ---------------------------------------------------------------------

(define sorted-eigen
	(sort
		(pti 'left-vec lit6) 
		(lambda (a b) (> (cdr a) (cdr b)))))

(define (print-tsv-pca port)
	(define cnt 0)
	(for-each
		(lambda (pr)
			(define w (car pr))
			(set! cnt (+ cnt 1))
			(format port "~d	\"~A\"	~A	~A	~A	~A	~A	~A\n" cnt (cog-name w)
				(lit w) (lit2 w) (lit3 w) (lit4 w) (lit5 w) (lit6 w)))
		(take sorted-eigen 40)))

(let ((outport (open-file "/tmp/power-iter.dat" "w")))
	(print-tsv-pca outport)
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
