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
; `make-pair-count-api` object, althought these are designed to be
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
;
; Extend the CNTOBJ with additional methods to compute wildcard counts
; for pairs, and store the results in the count-object.
; That is, compute the summations N(x,*) = sum_y N(x,y) where (x,y)
; is a pair, and N(x,y) is the count of how often that pair has been
; observed, and * denotes the wild-card, ranging over all items
; supported in that slot.
;
; The CNTOBJ needs to be an object implementing methods to get the
; support, and the supported pairs. So, the left-support is the set
; of all x's for which 0 < N(x,y) for some y.  Dual to the left-support
; are the right-stars, which is the set of all pairs (x,y) for any
; given, fixed x.
;
; The CNTOBJ needs to implement the 'left-support and 'right-support
; methods, to return these two sets, and also the 'left-stars and the
; 'right-stars methods, to return those sets.
;
; The CNTOBJ also needs to implement the setters, so that the wild-card
; counts can be cached. That is, the object must also have the
; 'set-left-wild-count, 'set-right-wild-count and 'set-wild-wild-count
; methods on it.
;
(define (make-compute-count CNTOBJ)
	(let ((cntobj CNTOBJ))

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
			(map cache-left-count (cntobj 'right-support)))

		(define (cache-all-right-counts)
			(map cache-right-count (cntobj 'left-support)))

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
				(cntobj 'left-support)))

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
				(cntobj 'right-support)))

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
; Extend the CNTOBJ with additional methods to compute observation
; frequencies and entropies for pairs, including partial-sum entropies
; (mutual information) for the left and right side of each pair.
; This will also cache the results of these computations in a
; standardized location.
;
; The CNTOBJ needs to be an object implementing methods to get pair
; observation counts, and wild-card counts (which must hold valid
; values). Specifically, it must have the 'pair-count, 'left-wild-count,
; 'right-wild-count and 'wild-wild-count methods on it.  Thus, if
; caching (which is the generic case) these need to have been computed
; and cached before using this class.

(define (make-compute-freq CNTOBJ)
	(let ((cntobj CNTOBJ)
			(tot-cnt 0))

		(define (init)
			(set! tot-cnt (cntobj `wild-wild-count)))

		; Compute the left-side wild-card frequency. This is the ratio
		; P(*,y) = N(*,y) / N(*,*) which gives the frequency at which
		; the pair (x,y) was observed.
		; This returns the frequency, or zero, if the pair was never
		; observed.
		(define (compute-left-freq ITEM)
			(/ (cntobj 'left-wild-count ITEM) tot-cnt))
		(define (compute-right-freq ITEM)
			(/ (cntobj 'right-wild-count ITEM) tot-cnt))

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

		; Compute and cache all of the left-side frequencies.
		; This computes P(*,y) for all y, in parallel.
		;
		; This method returns a list of all of the atoms holding
		; those counts; handy for storing in a database.
		(define (cache-all-left-freqs)
			(map cache-left-freq (cntobj 'right-support)))
		(define (cache-all-right-freqs)
			(map cache-right-freq (cntobj 'right-support)))

		; Methods on this class.
		(lambda (message . args)
			(case message
				((init-freq)             (init))
				((compute-left-freq)     (apply compute-left-freq args))
				((compute-right-freq)    (apply compute-right-freq args))
				((cache-left-freq)       (apply cache-left-freq args))
				((cache-right-freq)      (apply cache-right-freq args))
				((cache-all-left-freqs)  (cache-all-left-freqs))
				((cache-all-right-freqs) (cache-all-right-freqs))
				(else (apply cntobj      (cons message args))))
		))
)

; ---------------------------------------------------------------------
;
; Extend the CNTOBJ with additional methods to compute the mutual
; information of pairs.
;
; The CNTOBJ needs to be an object implementing methods to get pair
; observation frequencies, which must return valid values; i.e. must
; have been previously computed. Specifically, it must have the
; 'left-logli, 'right-logli and 'pair-logli methods.  For caching,
; it must also have the 'set-pair-mi method.
;
; The MI computations are done as a batch, looping over all pairs.

(define (make-batch-mi FRQOBJ)
	(let ((frqobj FRQOBJ))

		; Loop over all pairs, computing the MI for each. The loop
		; is actually two nested loops, with a loop over the
		; left-supports on the outside, and over right-stars for
		; the inner loop. This returns a list of all atoms holding
		; the MI, suitable for iterating for storage.
		(define (compute-n-cache-pair-mi)
			(define all-atoms '())
			(define n-rpt 0)
			(define lefties (frqobj 'left-support))
			(define nlefties (length lefties))

			(define (right-loop left-item)
				; Either of the get-loglis below may throw an exception,
				; if the particular item-pair doesn't have any counts.
				; This is rare, but can happen: e.g. (Any "left-word")
				; (Word "###LEFT-WALL###") will have zero counts.  This
				; would have an infinite logli and an infinite MI. So we
				; skip this, with a try-catch block.
				(catch #t (lambda ()
					(define r-logli (frqobj 'right-wild-logli left-item))
					(for-each
						(lambda (lipr)
							(define right-item (gdr lipr))
							(define l-logli (frqobj 'left-wild-logli right-item))
							(define pr-logli (frqobj 'pair-logli lipr))
							(define mi (- (+ r-logli l-logli) pr-logli))
							(define atom (frqobj 'set-pair-mi lipr mi))
							(set! all-atoms (cons atom all-atoms))
						)
						(frqobj 'right-stars left-item))

					; Print a progress report.
					(set! nrpt (+ nrpt 1))
					(if (eqv? 0 (mod nrpt 10000))
						(format #t "MI done ~A outer loops of ~A\n" nrpt nlefties))
					)
					(lambda (key . args) #f)) ; catch handler
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

		; Compute the total entropy for the set. This loops over all
		; pairs, and computes the sum
		;   H = sum_x sum_y p(x,y) log_2 p(x,y)
		; It returns a single numerical value, for the entire set.
		(define (compute-total-entropy)
			(define entropy 0)

			(define (right-loop left-item)
				(for-each
					(lambda (lipr)
						; The get-logli below may throw an exception, if
						; the particular item-pair doesn't have any counts.
						; XXX does this ever actually happen?  It shouldn't,
						;right?
						(catch #t (lambda ()
								(define pr-freq (frqobj 'pair-freq lipr))
								(define pr-logli (frqobj 'pair-logli lipr))
								(define h (* pr-freq pr-logli))
								(set! entropy (+ entropy h))
							)
							(lambda (key . args) #f))) ; catch handler
					(frqobj 'right-stars left-item)))

			(for-each right-loop (frqobj 'left-support))

			; Return the single number.
			entropy
		)

		; Methods on this class.
		(lambda (message . args)
			(case message
				((cache-pair-mi)         (compute-n-cache-pair-mi))
				((total-entropy)         (compute-total-entropy))
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
(define (batch-all-pair-mi OBJ)

	(define start-time (current-time))
	(define (elapsed-secs)
		(define diff (- (current-time) start-time))
		(set! start-time (current-time))
		diff)

	; Decorate the object with a counting API.
	(define obj-get-set-api (make-pair-count-api OBJ))

	; Decorate the object with methods that can compute counts.
	(define count-obj (make-compute-count obj-get-set-api))

	; Decorate the object with methods that can compute frequencies.
	(define freq-obj (make-compute-freq
		(make-pair-freq-api obj-get-set-api)))

	(format #t "Support: num left=~A num right=~A\n"
			(OBJ 'left-support-size)
			(OBJ 'right-support-size))

	; First, compute the summations for the left and right wildcard counts.
	; That is, compute N(x,*) and N(*,y) for the supports on x and y.

	(count-obj 'cache-all-left-counts)
	(count-obj 'cache-all-right-counts)

	(format #t "Done with wild-card count N(*,w) and N(w,*) in ~A secs\n"
		(elapsed-secs))

	; Now, compute the grand-total
	(store-atom (count-obj 'cache-total-count))
	(format #t "Done computing N(*,*) total-count=~A in ~A secs\n"
		(obj-get-set-api 'wild-wild-count)
		(elapsed-secs))

	(display "Start computing log P(*,w)\n")

	; Compute the left and right wildcard frequencies and
	; log-frequencies.
	(freq-obj 'init-freq)

	(let ((lefties (freq-obj 'cache-all-left-freqs)))
		(format #t "Done computing ~A left-wilds in ~A secs\n"
			(length lefties) (elapsed-secs))
		(for-each
			(lambda (atom) (if (not (null? atom)) (store-atom atom)))
			lefties)
		(format #t "Done storing ~A left-wilds in ~A secs\n"
			(length lefties) (elapsed-secs))
	)

	(display "Done with -log P(*,w), start -log P(w,*)\n")

	(let ((righties (freq-obj 'cache-all-right-freqs)))
		(format #t "Done computing ~A right-wilds in ~A secs\n"
			(length righties) (elapsed-secs))
		(for-each
			(lambda (atom) (if (not (null? atom)) (store-atom atom)))
			righties)
		(format #t "Done storing ~A right-wilds in ~A secs\n"
			(length righties) (elapsed-secs))
	)

	(display "Done computing -log P(w,*) and <-->\n")

	; Enfin, the word-pair mi's
	(display "Going to do individual word-pair MI\n")

	(let* ((bami (make-batch-mi freq-obj))
			(all-atoms (bami 'cache-pair-mi))
			(num-prs (length all-atoms)))

		; This is a fat wrapper around `store-atom` that simply prints
		; a progress report for the stores. The problem is that millions
		; of pairs may need to be stored, and this just takes a long time.
		(define store-cnt 0)
		(define start-time (current-time))
		(define (store-rpt ATOM)
			(store-atom ATOM)
			(set! store-cnt (+ 1 store-cnt))
			(if (eqv? 0 (mod store-cnt 100000))
				(begin
					(define elapsed (- (current-time) start-time))
					(set! start-time (current-time))
					(format #t "Stored ~A of ~A pairs in ~A secs\n"
						 store-cnt num-prs elapsed))))

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
