;
; compute-mi.scm
;
; Compute the mutual information of pairs of items.
;
; Copyright (c) 2013, 2014, 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute the mutual information held in pairs
; of "items".  The "items" can be any atoms, all of the same atom-type,
; arranged in ordered pairs via a ListLink.  For example,
;
;     ListLink
;          SomeAtom "left-side"
;          SomeAtom "right-hand-part"
;
; In the current usage, the SomeAtom is a WordNode, and the pairs are
; word-pairs obtained from linguistic analysis.  However, these scripts
; are general, and work for any kind of pairs, not just words.
;
; It is presumed that a database of counts of pairs has been already
; generated; these scripts work off of those counts.  We say "database",
; instead of "atomspace", because the scripts will automatically store
; the resulting counts in the (SQL) persistence backend, as they are
; computed.  This simplifies data management a little bit.
;
; It is assumed that all the count of all pair observations are stored
; as the "count" portion of the CountTruthValue on some link. For
; example, some (not not all!) of the linguistic word-pairs are stored
; as
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "some-word"
;         WordNode "other-word"
;
; In the general case, access to this structure is provided by methods
; on the "low-level API". These include:
;   'left-type and 'right-type, both of which should return 'WordNode
;         for the above.
;   'item-pair, which should return the EvaluationLink, given the
;        ListLink
;   'left-wildcard and 'right-wildcard, indicating where the partial
;        sums, such as N(x,*) and N(*,y) should be stored.

; Let N(wl,wr) denote the number of times that the pair (wl, wr) has
; actually been observed; that is, N("some-word", "other-word") for the
; example above.  Properly speaking, this count is conditioned on the
; LinkGrammarRelationshipNode "ANY", so the correct notation would be
; N(rel, wl, wr) with `rel` the relationship.  In what follows, the
; relationship is always assumed to be the same, and is thus dropped.
; (the relationship is provided through the GET-PAIR functiion).
;
; The mutual information for a pair is defined as follows:  Given
; two items, wl and wr, define three probabilities:
;
;    P(wl,wr) = N(wl,wr) / N(*,*)
;    P(wl,*)  = N(wl,*)  / N(*,*)
;    P(*,wr)  = N(*,wr)  / N(*,*)
;
; The N(*,*), N(wl,*) and  N(*,wr) are wild-card counts, and are defined
; to be sums over all observed left and right counts.  That is,
;
;    N(wl,*) = Sum_wr N(wl,wr)
;    N(*,wr) = Sum_wl N(wl,wr)
;    N(*,*) = Sum_wl Sum_wr N(wl,wr)
;
; These sums are computed, for a given item, by the `make-compute-count`
; object defined below.  It stores these counts at the locations
; provided by the underlying object. By default, thse are given by
; `add-pair-count-api` object, althought these are designed to be
; overloaded, if needed.

; For example, for word-pair counts, the wild-card sums are stored
; with the atoms
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         AnyNode "left-word"
;         WordNode "bird"
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         AnyNode "right-word"
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         AnyNode "left-word"
;         AnyNode "right-word"
;
; Here, AnyNode plays the role of *.  Thus, N(*,*) is shorthand for the
; last of these triples.
;
; After they've been computed, the values for N(*,y) and N(x,*) can be
; fetched with the 'left-wild-count and 'right-wild-count methods on
; the object.  The value for N(*,*) can be gotten with the
; 'wild-wild-count method.
;
; The mutual information for the pair (x,y) is defined as
;
;     MI(x,y) = - log_2 [ p(x,y) /  p(x,*) p(*,y) ]
;
; This is computed by the script batch-all-pair-mi below. The value is
; stored at the location provided by the 'set-pair-mi method on the
; object.  It can later be retrieved with the corresponding 'pair-mi
; method. Because everything is stored in the database, it is
; straight-forward to perform this batch computation only once,
; and then rely on the cached values fetched from the database.
;
; That's all there's to this.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (ice-9 threads))
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; A progress report utility.
; The wraps the FUNC function, and prints a progress report MSG
; every WHEN calls to FUNC.
; FUNC should be the function to be called, taking one argument.
; MSG should be a string of the form
;    "Did ~A of ~A in ~A seconds (~A items/sec)\n"
; WHEN should be how often to print (modulo)
; TOTAL should be the total number of items to process.

(define (make-progress-rpt FUNC WHEN TOTAL MSG)
	(let ((func FUNC)
			(when WHEN)
			(total TOTAL)
			(msg MSG)
			(cnt 0)
			(start-time 0))
		(lambda (item)
			; back-date to avoid divide-by-zero
			(if (eqv? 0 cnt) (set! start-time (- (current-time) 0.00001)))
			(func item)
			(set! cnt (+ 1 cnt))
			(if (eqv? 0 (modulo cnt when))
				(let* ((elapsed (- (current-time) start-time))
						(rate (/ (exact->inexact when) elapsed)))
					(format #t msg cnt total elapsed rate)
					(set! start-time (current-time))))))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
;
; Extend the LLOBJ with additional methods to compute wildcard counts
; for pairs, and store the results using the count-object API.
; That is, compute the summations N(x,*) = sum_y N(x,y) where (x,y)
; is a pair, and N(x,y) is the count of how often that pair has been
; observed, and * denotes the wild-card, ranging over all items
; supported in that slot.
;
(define (make-compute-count LLOBJ)

	; We need 'left-basis, provided by add-pair-stars
	; We need 'set-left-wild-count, provided by add-pair-count-api
	(let ((cntobj (add-pair-count-api (add-pair-stars LLOBJ))))

		; Compute the left-side wild-card count. This is the number
		; N(*,y) = sum_x N(x,y) where ITEM==y and N(x,y) is the number
		; of times that the pair (x,y) was observed.
		; This returns the count, or zero, if the pair was never observed.
		(define (compute-left-count ITEM)
			(fold
				(lambda (pr sum) (+ sum (cntobj 'pair-count pr)))
				0
				(cntobj 'left-stars ITEM)))

		; Compute and cache the left-side wild-card counts N(*,y).
		; This returns the atom holding the cached count, thus
		; making it convient to persist (store) this cache in
		; the database. It returns nil if the count was zero.
		(define (cache-left-count ITEM)
			(define cnt (compute-left-count ITEM))
			(if (< 0 cnt)
				(cntobj 'set-left-wild-count ITEM cnt)
				'()))

		; Compute the right-side wild-card count N(x,*).
		(define (compute-right-count ITEM)
			(fold
				(lambda (pr sum) (+ sum (cntobj 'pair-count pr)))
				0
				(cntobj 'right-stars ITEM)))

		; Compute and cache the right-side wild-card counts N(x,*).
		; This returns the atom holding the cached count, or nil
		; if the count was zero.
		(define (cache-right-count ITEM)
			(define cnt (compute-right-count ITEM))
			(if (< 0 cnt)
				(cntobj 'set-right-wild-count ITEM cnt)
				'()))

		; Compute and cache all of the left-side wild-card counts.
		; This computes N(*,y) for all y, in parallel.
		;
		; This method returns a list of all of the atoms holding
		; those counts; handy for storing in a database.
		(define (cache-all-left-counts)
			(map cache-left-count (cntobj 'right-basis)))

		(define (cache-all-right-counts)
			(map cache-right-count (cntobj 'left-basis)))

		; Compute the total number of times that all pairs have been
		; observed. In formulas, return
		;     N(*,*) = sum_x N(x,*) = sum_x sum_y N(x,y)
		;
		; This method assumes that the partial wild-card counts have
		; been previously computed and cached.  That is, it assumes that
		; the 'right-wild-count returns a valid value, which really
		; should be the same value as 'compute-right-count on this object.
		(define (compute-total-count-from-left)
			(fold
				;;; (lambda (item sum) (+ sum (compute-right-count item)))
				(lambda (item sum) (+ sum (cntobj 'right-wild-count item)))
				0
				(cntobj 'left-basis)))

		; Compute the total number of times that all pairs have been
		; observed. That is, return N(*,*) = sum_y N(*,y). Note that
		; this should give exactly the same result as the above; however,
		; the order in which the sums are performed is distinct, and
		; thus any differences indicate a bug.
		(define (compute-total-count-from-right)
			(fold
				;;; (lambda (item sum) (+ sum (compute-left-count item)))
				(lambda (item sum) (+ sum (cntobj 'left-wild-count item)))
				0
				(cntobj 'right-basis)))

		; Compute the total number of times that all pairs have been
		; observed. That is, return N(*,*).  Throws an error if the
		; left and right summations fail to agree.
		(define (compute-total-count)
			(define l-cnt (compute-total-count-from-left))
			(define r-cnt (compute-total-count-from-right))

			; The left and right counts should be equal!
			(if (not (eqv? l-cnt r-cnt))
				(throw 'bad-summation 'count-all-pairs
					(format #f "Error: pair-counts unequal: ~A ~A\n" l-cnt r-cnt)))
			l-cnt)

		; Compute and cache the total observation count for all pairs.
		; This returns the atom holding the cached count.
		(define (cache-total-count)
			(define cnt (compute-total-count))
			(cntobj 'set-wild-wild-count cnt))

		; Methods on this class.
		(lambda (message . args)
			(case message
				((compute-left-count)     (apply compute-left-count args))
				((cache-left-count)       (apply cache-left-count args))
				((compute-right-count)    (apply compute-right-count args))
				((cache-right-count)      (apply cache-right-count args))
				((cache-all-left-counts)  (cache-all-left-counts))
				((cache-all-right-counts) (cache-all-right-counts))
				((compute-total-count)    (compute-total-count))
				((cache-total-count)      (cache-total-count))
				(else (apply cntobj (cons message args))))
			))
)

; ---------------------------------------------------------------------
;
; Extend the LLOBJ with additional methods to compute observation
; frequencies and entropies for pairs, including partial-sum entropies
; (mutual information) for the left and right side of each pair.
; This will also cache the results of these computations in a
; standardized location.
;
; The LLOBJ must have valid left and right wild-card counts on it.
; These need to have been previously computed, before methods on
; this class are called.
;
; Before using this class, the 'init-freq method must be called,
; and it must be called *after* a valid wild-wild count is available.

(define (make-compute-freq LLOBJ)

	; We need 'left-basis, provided by add-pair-stars
	; We need 'wild-wild-count, provided by add-pair-count-api
	; We need 'set-left-wild-freq, provided by add-pair-freq-api
	(let ((cntobj (add-pair-freq-api (add-pair-count-api
					(add-pair-stars LLOBJ))))
			(tot-cnt 0))

		(define (init)
			(set! tot-cnt (cntobj `wild-wild-count)))

		; Compute the pair frequency P(x,y) = N(x,y) / N(*,*)  This is
		; the frequency with which the pair (x,y) is observed. Return
		; the frequency, or zero, if the pair was never observed.
		(define (compute-pair-freq PAIR)
			(/ (cntobj 'pair-count PAIR) tot-cnt))

		; Compute the left-side wild-card frequency. This is the ratio
		; P(*,y) = N(*,y) / N(*,*) = sum_x P(x,y)
		(define (compute-left-freq ITEM)
			(/ (cntobj 'left-wild-count ITEM) tot-cnt))
		(define (compute-right-freq ITEM)
			(/ (cntobj 'right-wild-count ITEM) tot-cnt))

		; Compute and cache the pair frequency.
		; This returns the atom holding the cached count, thus
		; making it convient to persist (store) this cache in
		; the database. It returns nil if the count was zero.
		(define (cache-pair-freq PAIR)
			(define freq (compute-pair-freq PAIR))
			(if (< 0 freq)
				(cntobj 'set-pair-freq PAIR freq)
				'()))

		; Compute and cache the left-side wild-card frequency.
		; This returns the atom holding the cached count, thus
		; making it convient to persist (store) this cache in
		; the database. It returns nil if the count was zero.
		(define (cache-left-freq ITEM)
			(define freq (compute-left-freq ITEM))
			(if (< 0 freq)
				(cntobj 'set-left-wild-freq ITEM freq)
				'()))

		(define (cache-right-freq ITEM)
			(define freq (compute-right-freq ITEM))
			(if (< 0 freq)
				(cntobj 'set-right-wild-freq ITEM freq)
				'()))

		; Compute and cache all of the pair frequencies.
		; This computes P(x,y) for all (x,y)
		; This returns a count of the pairs.
		(define (cache-all-pair-freqs)
			(define cnt 0)
			(define lefties (cntobj 'left-basis))
			(define (right-loop left-item)
				(for-each
					(lambda (pr)
						(cache-pair-freq pr)
						(set! cnt (+ cnt 1)))
					(cntobj 'right-stars left-item)))

			(for-each right-loop lefties)
			cnt)

		; Compute and cache all of the left-side frequencies.
		; This computes P(*,y) for all y, in parallel.
		;
		; This method returns a list of all of the atoms holding
		; those counts; handy for storing in a database.
		(define (cache-all-left-freqs)
			(map cache-left-freq (cntobj 'right-basis)))
		(define (cache-all-right-freqs)
			(map cache-right-freq (cntobj 'left-basis)))

		; Methods on this class.
		(lambda (message . args)
			(case message
				((init-freq)             (init))

				((compute-pair-freq)     (apply compute-pair-freq args))
				((compute-left-freq)     (apply compute-left-freq args))
				((compute-right-freq)    (apply compute-right-freq args))

				((cache-pair-freq)       (apply cache-pair-freq args))
				((cache-left-freq)       (apply cache-left-freq args))
				((cache-right-freq)      (apply cache-right-freq args))

				((cache-all-pair-freqs)  (cache-all-pair-freqs))
				((cache-all-left-freqs)  (cache-all-left-freqs))
				((cache-all-right-freqs) (cache-all-right-freqs))

				(else (apply cntobj      (cons message args))))
		))
)

; ---------------------------------------------------------------------
;
; Extend the LLOBJ with additional methods to compute the mutual
; information of all pairs; each pair is then tagged with the resulting
; MI. (This is the "cache" -- the resulting MI is "cached" with the
; atom).
;
; The LLOBJ object must have valid pair-frequencies on it, accessible
; by the standard frequency API. These need to have been pre-computed,
; before using this object.
;
; The MI computations are done as a batch, looping over all pairs.

(define (make-batch-mi LLOBJ)

	(define start-nsecs (get-internal-real-time))
	(define (elapsed-millisecs)
		(define diff (- (get-internal-real-time) start-nsecs))
		(set! start-nsecs (get-internal-real-time))
		(/ (* diff 1000.0) internal-time-units-per-second))

	; We need 'left-basis, provided by add-pair-stars
	; We need 'pair-freq, provided by add-pair-freq-api
	; We need 'set-pair-mi, provided by add-pair-freq-api
	; We need 'right-wild-count, provided by add-pair-count-api
	(let ((star-obj (add-pair-stars LLOBJ))
			(cntobj (add-pair-count-api LLOBJ))
			(frqobj (add-pair-freq-api LLOBJ)))

		; Loop over all pairs, computing the MI for each. The loop
		; is actually two nested loops, with a loop over the
		; left-basis on the outside, and over right-stars for
		; the inner loop. This returns a list of all atoms holding
		; the MI, suitable for iterating for storage.
		(define (compute-n-cache-pair-mi)
			(define all-atoms '())
			(define lefties (star-obj 'left-basis))

			; progress stats
			(define cnt-pairs 0)
			(define cnt-lefties 0)
			(define nlefties (length lefties))

			(define (right-loop left-item)

				; Check for non-zero counts. A zero here will cause the
				; 'right-wild-logli to throw. The problem here is that
				; every throw gets logged into the logfile (currently, they
				; are not silent) which can sometimes be a huge performance
				; hit. So avoid the throws.
				; Anyway: zero counts means undefined MI.
				(if (< 0 (cntobj 'right-wild-count left-item))
					(let ((r-logli (frqobj 'right-wild-logli left-item)))

						; Compute the MI for exactly one pair.
						(define (do-one-pair lipr)
							(define pr-freq (frqobj 'pair-freq lipr))
							(define pr-logli (frqobj 'pair-logli lipr))

							(define right-item (gdr lipr))
							(if (< 0 (frqobj 'left-wild-count right-item))
								(let* ((l-logli (frqobj 'left-wild-logli right-item))
										(fmi (- pr-logli (+ r-logli l-logli)))
										(mi (* pr-freq fmi))
										(atom (frqobj 'set-pair-mi lipr mi fmi)))
									(set! all-atoms (cons atom all-atoms))
									(set! cnt-pairs (+ cnt-pairs 1)))))

						; Run the inner loop
						(for-each
							do-one-pair
							(star-obj 'right-stars left-item))

						; Print some progress statistics.
						(set! cnt-lefties (+ cnt-lefties 1))
						(if (eqv? 0 (modulo cnt-lefties 10000))
							(format #t "Done ~A of ~A outer loops, pairs=~A\n"
								cnt-lefties nlefties cnt-pairs))

					))
			)

			;; XXX Maybe FIXME This could be a par-for-each, to run the
			; calculations in parallel, but then we need to make the
			; all-atoms list thread-safe.  Two problems: one is that
			; current guile par-for-each implementation sucks.
			; The other is that the atom value-fetching is done under
			; a global lock, thus effectively single-threaded.
			(for-each right-loop lefties)

			; Return the list of ALL atoms with MI on them
			all-atoms
		)

		; Methods on this class.
		(lambda (message . args)
			(case message
				((cache-pair-mi)         (compute-n-cache-pair-mi))
				(else (apply frqobj      (cons message args))))
		))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
;
; Compute the mutual information between all pairs. Counts, frequencies
; and left, right partial sums are also performed; this is an all-in-one
; routine, which computes all of the needed pre-requisites, and stores
; them, as well as the MI, in the database.
;
; The mutual information between pairs is described in the overview,
; up top of this file. The access to the pairs is governed by the
; the methods on the passed object.
;
; Among the things that are computed and stored are the partial sums
; of counts, i.e. the N(x,*) and N(*,y) explained up top, the total
; count N(*,*), the frequencies p(x,y) = N(x,y) / N(*,*), the
; corresponding partial sums.  All of these quantities are written
; back to the database, at the time of computation.
;
; In order to work correctly, this function assumes that the object
; has at least the minimal low-level API to identify where to find
; the counts on pairs.  This script is designed to work with any kinds
; of pairs.
;
; Running this script can take hours or longer, depending on the size
; of the dataset. Progress reports are printed to stdout, including
; timing and summary statistics. This script wasn't really designed to
; be efficient; instead, the goal to to allow general, generic knowledge
; representation.  You can compute MI between any kinds of things
; If you just need to count one thing, writing custom scripts that do
; NOT use the atomspace would almost surely be faster.  We put up with
; the performance overhead here in order to get the flexibility that
; the atomspace provides.
;
(define-public (batch-all-pair-mi OBJ)

	(define start-time (current-time))
	(define (elapsed-secs)
		(define diff (- (current-time) start-time))
		(set! start-time (current-time))
		diff)

	; Decorate the object with methods that report support.
	(define wild-obj (add-pair-stars OBJ))

	; Decorate the object with methods that can compute counts.
	(define count-obj (make-compute-count OBJ))

	; Decorate the object with methods that can compute frequencies.
	(define freq-obj (make-compute-freq OBJ))

	; Decorate the object with methods that can compute the pair-MI.
	(define batch-mi-obj (make-batch-mi OBJ))

	(format #t "Support: num left=~A num right=~A\n"
			(length (wild-obj 'left-basis))
			(length (wild-obj 'right-basis)))

	; First, compute the summations for the left and right wildcard counts.
	; That is, compute N(x,*) and N(*,y) for the supports on x and y.

	(count-obj 'cache-all-left-counts)
	(count-obj 'cache-all-right-counts)

	(format #t "Done with wild-card count N(x,*) and N(*,y) in ~A secs\n"
		(elapsed-secs))

	; Now, compute the grand-total.
	(store-atom (count-obj 'cache-total-count))
	(format #t "Done computing N(*,*) total-count=~A in ~A secs\n"
		((add-pair-count-api OBJ) 'wild-wild-count)
		(elapsed-secs))

	; Compute the pair-frequencies, and the left and right
	; wildcard frequencies and log-frequencies.
	(freq-obj 'init-freq)

	(display "Going to do individual pair frequencies\n")
	(let ((pair-cnt (freq-obj 'cache-all-pair-freqs)))
		(format #t "Done computing ~A pairs in ~A secs\n"
				pair-cnt (elapsed-secs)))

	(display "Start computing log P(*,y)\n")
	(let ((lefties (freq-obj 'cache-all-left-freqs)))

		(define store-rpt
			(make-progress-rpt store-atom 40000  (length lefties)
				"Stored ~A of ~A lefties in ~A secs (~A stores/sec)\n"))

		(format #t "Done computing ~A left-wilds in ~A secs\n"
			(length lefties) (elapsed-secs))
		(for-each
			(lambda (atom) (if (not (null? atom)) (store-rpt atom)))
			lefties)
		(format #t "Done storing ~A left-wilds in ~A secs\n"
			(length lefties) (elapsed-secs))
	)

	(display "Done with -log P(*,y), start -log P(x,*)\n")

	(let ((righties (freq-obj 'cache-all-right-freqs)))
		(format #t "Done computing ~A right-wilds in ~A secs\n"
			(length righties) (elapsed-secs))
		(for-each
			(lambda (atom) (if (not (null? atom)) (store-atom atom)))
			righties)
		(format #t "Done storing ~A right-wilds in ~A secs\n"
			(length righties) (elapsed-secs))
	)

	(display "Done computing -log P(x,*) and P(*,y)\n")

	; Enfin, the pair mi's
	(display "Going to do individual pair MI\n")

	(let* ((all-atoms (batch-mi-obj 'cache-pair-mi))
			(num-prs (length all-atoms)))

		; Create a wrapper around `store-atom` that prints a progress
		; report.  The problem is that millions of pairs may need to be
		; stored, and this just takes a long time.
		(define store-rpt
			(make-progress-rpt store-atom 100000 num-prs
				"Stored ~A of ~A pairs in ~A secs (~A pairs/sec)\n"))

		; This print triggers as soon as the let* above finishes.
		(format #t "Done computing ~A pair MI's in ~A secs\n"
			num-prs (elapsed-secs))
		(for-each store-rpt all-atoms)
		(format #t "Done storing ~A pair MI's in ~A secs\n"
			num-prs (elapsed-secs))
	)

	(display "Finished with MI computations\n")
)

; ---------------------------------------------------------------------
