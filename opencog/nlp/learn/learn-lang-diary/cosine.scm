;
; Notes and tools that were used to create the cosine-similarity
; explorations in the diary, circa June, July 2017.
; See also the 'notes' file one directory down.

(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog matrix))

; The database:
(sql-open "postgres:///en_pairs_ttwo_sim?user=linas")
(fetch-all-words)

; psedo-csets aka sectons.
(define pca (make-pseudo-cset-api))
(define psa (add-pair-stars pca))
(psa 'fetch-pairs)

; Verify that the loaded data is as claimed:
(print-matrix-summary-report psa)
; Gives:
;; Summary Report for Correlation Matrix Word-Disjunct Pairs (Connector Sets)
;; Left type: WordNode    Right Type: ConnectorSeq    Pair Type: Section
;; Wildcard: (ListLink (ctv 0 0 14382276)
;;    (AnyNode "cset-word")
;;    (AnyNode "cset-disjunct")
;; )
;; Rows: 175559 Columns: 3401462
;; Size: 6438484 non-zero entries of 597157267258 possible
;; Fraction non-zero: 1.0782E-5 Sparsity (-log_2): 16.501
;; Total observations: 14382276.0  Avg obs per pair: 2.2338
;; Entropy Total: 21.006   Left: 14.906   Right: 10.007
;; Total MI: -3.906
;; 
;;                  Left         Right     Avg-left     Avg-right
;;                  ----         -----     --------     ---------
;; Support (l_0)  6.1010E+4    9176.
;; Count   (l_1)  1.5853E+5    6.4221E+4     2.598        6.999
;; Length  (l_2)  5694.        3476.         9.3332E-2    .3788
;; RMS Count      5666.        3444.         9.2870E-2    .3753


; Apply a filter to the dataset; this is the filter in the diary.
; See circa line 9700 of the 'notes' file.
(define fsi (add-subtotal-filter psa 50 30 10))

; All of the words that participate in connector-sets.
(define all-cset-words (fsi 'left-basis))
(length all-cset-words)  ; 13122

; 803 words that were observed 1500 times, or more
(define top-cset-words
   (filter (lambda (wrd) (< 1500 (cset-vec-word-observations wrd)))
		all-cset-words))

(length top-cset-words) ; 803

; Rank them according to the number of observations.
(define ranked-csw (sort top-cset-words
   (lambda (a b) (> (cset-vec-word-observations a) (cset-vec-word-observations b)))))

; OK, cosine time
(define poi (add-pair-cosine-compute fsi))
; Usage: (poi 'right-cosine WORD-A WORD-B)

; notes, circa line 10540

; -----------------------------------------------------------------

(define (pair-sym-cache wa wb KEY FN)
"
	Get cached value, or compute it.  Symmetric.
	Stores the value in the atomspace, under KEY for (ListLink wa wb)
"
	(define (get-val PR)
		(cog-value-ref (cog-value PR KEY) 0))
	(define (set-val PR VAL)
		(cog-set-value! PR KEY (FloatValue VAL)))

	; Where to check for values
	(define wpr (ListLink wa wb))

	; The value, or #f
	(define got
		(catch #t
			(lambda () (get-val wpr))
			(lambda (k . args) #f)))

	; If the value is #f, then try the reversed pair.
	(if got got
		(let* ((flp (ListLink wb wa))
				(gat (catch #t
					(lambda () (get-val flp))
					(lambda (k . args) #f))))
			(if gat
				; Found the reversed pair; record the forward pair
				(begin
					(set-val wpr gat)
					gat)
				; Found neither foreward nor reversed. Compute it.
				(let ((val (FN wa wb)))
					(set-val wpr val)
					(set-val flp val)
					val)))))

(define (left-sum ITEM LST FN)
"
 Perform the wild-card sum over FN(*, ITEM) for * in LST
"
	(define (summer ITEM)
		(fold
			(lambda (it acc) (+ acc (FN it ITEM)))
			0
			LST))
	(make-afunc-cache summer))


