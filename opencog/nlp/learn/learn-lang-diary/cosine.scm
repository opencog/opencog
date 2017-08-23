;
; Notes and tools that were used to create the cosine-similarity
; explorations in the diary, circa June, early July 2017.
; See also the 'notes' file one directory down.
;
; Hmm. obsolete. Had to redo the stats, and did not use this file.
; used the stuff in disjunct-stats.scm instead.  The cosing calcs
; got moved into generic (opencog matrix) module.

(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog matrix))
(use-modules (srfi srfi-1))
(use-modules (opencog cogserver))
(start-cogserver)


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

; notes, circa line 10556

; Compute, and cache in the atomspace, the cosines.
(define (get-cos wa wb)
	(define cos-fkey (PredicateNode "*-Cosine 803 Key-*"))
	(pair-sym-cache wa wb cos-fkey
		(lambda (wx wy) (poi 'right-cosine wx wy))))

; Compute, and SQL-store the cosines.
(define (make-cos wa wb)
	(define cos-fkey (PredicateNode "*-Cosine 803 Key-*"))
	(pair-sym-cache wa wb cos-fkey
		(lambda (wx wy)
			(define wcos (poi 'right-cosine wx wy))
			(store-atom (List wx wy))
			(store-atom (List wy wx))
			wcos)))

; Compute cosines for the N words in the list.
(define (make-all-cos wordlist)
	(define wlen (length wordlist))
	(if (< 0 wlen)
		(let ((head (car wordlist))
				(rest (cdr wordlist)))
			(format #t "Pairs remaining: ~A\n" wlen)
			(for-each (lambda (w) (make-cos head w)) wordlist)
			(make-all-cos rest))))

(make-all-cos (take ranked-csw 40))

; Compute all 803 of them, in due time.  This takes days.
(define (make-them-all nxt)
	(if (< nxt (length ranked-csw))
		(make-all-cos (take ranked-csw nxt))
		(make-all-cos ranked-csw))
	(if (< nxt (length ranked-csw))
		(make-them-all (inexact->exact (truncate (* 1.5 nxt))))))


; Return a function that computes the cosine mutual information 
(define (make-get-cmi LST)
	(define left-marg (make-left-summer LST get-cos))
	(define right-marg (make-right-summer LST get-cos))
	(define cos-wild-wild
		(fold (lambda (it acc) (+ acc (left-marg it))) 0 LST))
	(define oln2 (- (/ 1.0 (log 2.0))))
	(lambda (word-a word-b)
		(* oln2 (log (/ 
			(* (get-cos word-a word-b) cos-wild-wild)
			(* (left-marg word-b) (right-marg word-a)))))))

(define get-cmi (make-get-cmi (take ranked-csw 108)))
(define get-cmi (make-get-cmi (take ranked-csw 162)))


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

(define (make-left-summer LST FN)
"
 Return a function (func ITEM) that performs wild-card sums
 over FN(*, ITEM) for * in LST.
"
	(define (summer ITEM)
		(fold
			(lambda (it acc) (+ acc (FN it ITEM)))
			0
			LST))
	(make-afunc-cache summer))


(define (make-right-summer LST FN)
"
 Return a function (func ITEM) that performs wild-card sums
 over FN(ITEM, *) for * in LST.
"
	(define (summer ITEM)
		(fold
			(lambda (it acc) (+ acc (FN ITEM it)))
			0
			LST))
	(make-afunc-cache summer))

(define (make-sym-pairs LST FN)
"
 Return a list of pair-value pairs of pairs constructed from LST
 and values obtained by applying FN.

 That is, return a list of ( (x . y) . v) where x and y are from LST
 and v is equal to (FN x y)
"
	(define (make-all-helper WLST RSLT)
		(define wlen (length WLST))
		(if (< 0 wlen)
			(let* ((head (car WLST))
					(rest (cdr WLST))
					(prs (map (lambda (w) (cons (cons head w) (FN head w))) WLST))
				)
				(format #t "Pairs remaining: ~A\n" wlen)
				(make-all-helper rest (append prs RSLT)))
			RSLT))
	(make-all-helper LST '()))
