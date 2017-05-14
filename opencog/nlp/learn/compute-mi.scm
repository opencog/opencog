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
; word-pairs obtained from lingistic analysis.  However, these scripts
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
; In the general case, the 'WordNode is actually of ITEM-TYPE.
; The actual atom holding the count is obtained by calling an
; access function: i.e. given the ListLink holding a pair, the
; GET-PAIR function returns a list of atoms holding the count.
; It is presumed that the total count is the sum over the counts
; all atoms in the list.
;
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
; These sums are computed, for a given item, by compute-pair-wildcard-counts
; below, and are computed for all items by batch-all-pair-wildcard-counts.
; The resulting counts are stored as the 'count' value on the
; CountTruthValue on the atoms provided by the GET-LEFT-WILD, the
; GET-RIGHT-WILD and the GET-WILD-WILD functions. For example, for word-pair
; counts, these will be the atoms
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
; After they've been computed, the values for N(w,*) and N(*,w) can be
; fetched with the `get-left-count-str` and `get-right-count-str`
; routines, below.  The value for N(*,*) can be gotten by calling
; `total-pair-observations`.
;
; In addition to computing and storing the probabilities P(wl,wr), it
; is convenient to also store the entropy or "log likelihood" of the
; probabilities. Thus, the quantity H(wl,*) = -log_2 P(wl,*) is computed.
; Both the probability, and the entropy are stored, under the key of
; (PredicateNode "*-FrequencyKey-*").
; Note the minus sign: the entropy H(wl,*) is positive, and gets larger,
; the smaller P is. Note that the logarithm is base-2.  In the scripts
; below, the phrase 'logli' is used as a synonym for this entropy.
;
; The mutual information between a pair of items is defined as
;
;     MI(wl,wr) = -(H(wl,wr) - H(wl,*) - H(*,wr))
;
; This is computed by the script batch-all-pair-mi below. The value is
; stored under the key of (Predicate "*-Pair MI Key-*") as a single
; float.
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
;
; The CNTOBJ needs to be an object implementing methods to get wild-card
; lists, i.e. the 'left-stars and 'right-stars methods, and also
; implementing setters, so that the wild-card counts can be cached.
; That is, the object must also have the 'set-left-wild-count,
; 'set-right-wild-count and 'set-wild-wild-count methods on it.
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

		; Compute and cache the left-side wild-card counts.
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

		; Compute and cache the right-side wild-card counts.
		; This returns the atom holding the cached count, or nil
		; if the count was zero.
		(define (cache-right-count ITEM)
			(define cnt (compute-right-count ITEM))
			(if (< 0 cnt)
				(cntobj 'set-right-wild-count ITEM cnt)
				'()))

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
				((compute-left-count)    (apply compute-left-count args))
				((cache-left-count)      (apply cache-left-count args))
				((compute-right-count)   (apply compute-right-count args))
				((cache-right-count)     (apply cache-right-count args))
				((compute-total-count)   (compute-total-count))
				((cache-total-count)     (cache-total-count))
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

		; Methods on this class.
		(lambda (message . args)
			(case message
				((init)               (init))
				((compute-left-freq)  (apply compute-left-freq args))
				((compute-right-freq) (apply compute-right-freq args))
				((cache-left-freq)    (apply cache-left-freq args))
				((cache-right-freq)   (apply cache-right-freq args))
				(else (apply cntobj   (cons message args))))
		))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Compute the left and right word-pair wildcard counts.
; That is, compute the summations N(w,*) and N(*,w) where * denotes
; a wildcard, and ranges over all words observed in that slot.
; Store the resulting counts in a wild-card count structure, described
; below.
;
; To be precise, the summation is performed relative to the given
; LinkGrammar relationship node.  That is, the sumation only occurs
; over word pairs connected by the given link-grammar link.
; (Link grammar links are encoded with LinkGrammarRelationshipNode's).
;
; Thus, a word pair is currently represented as:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         WordNode "bird"
;
; To compute the left and right counts, we do an ad-hoc pattern
; match to the pattern below (ad-hoc because we don't bother with the
; pattern matcher here, the patten is too simple. In other cases, for
; structures more complex than word-pairs, we will need the matcher...)
; (Err, actually, we *do* use the pattern matcher, but the code would
; be faster and more efficient if we didn't. This should be fixed...)
;
; The match pattern for the right-counts is:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         VariableNode of type WordNode   ; i.e. a wildcard here.
;
; while that for left-counts is:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         VariableNode of type WordNode  ; i.e. a wildcard on the left.
;         WordNode "bird"
;
; Sums are performed over all matching patterns (i.e. all values of the
; VariableNode).
;
; The resulting sums are stored in the CountTruthValues (on the
; EvaluationLink) of the following structures:
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
; This routine assumes that all relevant atoms are already in the
; atomspace. If they're not, incorrect counts will be obtained. Either
; batch-fetch all word-pairs, or use the
; `fetch-and-compute-pair-wildcard-counts` routine, below, to fetch
; the individual word.
;
; Returns the two wild-card EvaluationLinks

(define (compute-pair-wildcard-counts OBJ ITEM)

	(define lefty (OBJ 'cache-left-count ITEM))
	(define righty (OBJ 'cache-right-count ITEM))

	(if (not (null? lefty)) (store-atom lefty))
	(if (not (null? righty)) (store-atom righty))
)

; ---------------------------------------------------------------------
; Compute the log-liklihood for all wild-card wordpairs.
;
; This assumes that wild-card word-pair counts have already been
; performed. This takes three functions as an argument, and a list
; of words.
;
; The GET-LEFT-WILD and GET-RIGHT-WILD functions should return
; an atom of the general form:
;
;   FooBarEvaluationLink
;      FooPredicateNode "Blah"
;      ListLink
;         WordNode "some-word"
;         AnyNode "right-word"
;
; and also for the flipped version (exchange WordNode and AnyNode)
; For example, FooBarEvaluationLink can be EvaluationLink, and
; FooPredicateNode can be LinkGrammarRelationshipNode "ANY".
;
; The wild-card counts will be fetched from the above atoms; the
; computed log-liklihood will be stored on the aove atoms.
;
; The GET-WILD-WILD function should return the atom holding
; the total wildcard count (i.e. N(R, *,*))
;
; The computation is performed "batch style", it loops over the list
; of words in ALL-WORDS; it assumes that assumes that all wild-card
; counts are up-to-date in the atomspace; no fetching from the database
; is performed.

(define (batch-all-pair-wildcard-logli OBJ ALL-WORDS)

	; Get the word-pair grand-total
	(define pair-total (OBJ 'wild-wild-count))

	; For each wild-card pair associated with the word,
	; obtain the log likelihood.
	(for-each
		(lambda (word)
			(let ((lefty (OBJ 'left-wildcard word))
					(righty (OBJ 'right-wildcard word)))

				; log-likelihood for the left wildcard
				(if (< 0 (get-count lefty))
					(store-atom (compute-atom-logli lefty pair-total)))

				; log-likelihood for the right wildcard
				(if (< 0 (get-count righty))
					(store-atom (compute-atom-logli righty pair-total)))))

		ALL-WORDS)
)

; ---------------------------------------------------------------------
;
; Compute word-pair mutual information, for all word-pairs with the
; given word being on the right.  This is a helper routine, to split
; up the double-loop over left and right words into two. This is the
; inner loop, looping over all left-words.
;
; The mutual information, and where its stored, is described in the
; overview, up top.  This routine is a batch routine; it assumes that
; all needed pairs are already in the atomspace (it does NOT fetch
; from storage). It assumes that the wild-card entropies (logli's) have
; already been computed.
;
;
(define (compute-pair-mi OBJ RIGHT-ITEM)

	; Get the word-pair grand-total
	(define pair-total (OBJ 'wild-wild-count))

	; Either of the get-loglis below will throw an exception, if
	; the particular word-pair doesn't have any counts. This is rare,
	; but can happen: e.g. (Any "left-word") (Word "###LEFT-WALL###")
	; will have zero counts.  This would have an infinite logli
	; an an infinite MI. So we skip this, with a try-catch block.
	(catch #t (lambda ()
	(let* (
			; left-stars are all the ListLinks in which the RIGHT-ITEM
			; appears on the right (and anything on the left)
			(left-stars (OBJ 'left-stars RIGHT-ITEM))

			; left-evs are the EvaluationLinks above the left-stars
			; That is, they have the wild-card in the left-hand slot.
			(left-evs (map! (lambda (lnk) (OBJ 'item-pair lnk)) left-stars))

			(l-logli (get-logli (OBJ 'left-wildcard RIGHT-ITEM)))
		)
		(for-each

			; This lambda sets the mutual information for each word-pair.
			(lambda (pair)
				(catch #t (lambda ()
					(let* (
							; the left-word of the word-pair
							(left-word (gadr pair))

							(r-logli (get-logli (OBJ 'right-wildcard left-word)))

							; Compute the logli log_2 P(l,r)/P(*,*)
							(atom (compute-atom-logli pair pair-total))

							; Get the log liklihood computed immediately above.
							(ll (get-logli atom))

							; Subtract the left and right entropies to get the
							; mutual information (at last!)
							(mi (- (+ l-logli r-logli) ll))
						)
						; Save the hard-won MI to the database.
						(store-atom (set-mi atom mi))
					))
				(lambda (key . args) #f)) ; catch handler
			)
			left-evs
		)
	))
	(lambda (key . args) #f)) ; catch handler
)

; ---------------------------------------------------------------------
;
; Compute the mutual information between all pairs.
;
; The mutual information between pairs is described in the overview,
; up top of this file. The access to the pairs is governed by the
; the methods on the assed object.
;
; The `all-singletons` argument should be a list of all items over
; which the pairs will be computed. Every item in this list should
; be of  (OBJ 'item-type)
; XXX FIXME .. which ones .. left or right????? XXX
;
; The algorithm uses a doubley-nested loop to walk over all pairs,
; in a sparse-matrix fashion: The outer loop is over all all items,
; the inner loop is over the incoming set of the items, that incoming
; set being composed of ListLinks that hold pairs. The ITEM-TYPE
; is used for filtering, to make sure that only valid pairs are
; accessed.
;
; Partial sums of counts, i.e. the N(w,*) and N(*,w) explained up top,
; are stored with the atoms that GET-LEFT-WILD and GET-RIGHT-WILD
; provide. The GET-WILD-WILD function returns the atom where N(*,*) is
; stored.
;
; The wild-card entropies and MI values are written back to the database
; as soon as they are computed, so as not to be lost.  The double-nested
; sums are distributed over all CPU cores, using guile's par-for-each,
; and can thus be very CPU intensive.
;
; Running this script can take hours, or longer (days?) depending on the
; size of the dataset.  This script wasn't really designed to be
; efficient; instead, the goal to to allow general, generic knowledge
; representation.  You can compute MI between any kind of thing.
; If you just need to count one thing, writing custom scripts that do
; NOT use the atomspace would almost surely be faster.  We put up with
; the performance overhead here in order to get the flexibility that
; the atomspace provides.
;
(define (batch-all-pair-mi OBJ all-singletons)

	; Decorate the object with methods that can compute things.
	(define comp-obj
		(make-compute-count
			(make-pair-count-api OBJ)))

	(define msg (format #f "Start counting, num words=~A\n"
			(length all-singletons)))
	(trace-msg msg)
	(display msg)

	; First, get the left and right wildcard counts.
	; That is, compute N(w,*) and N(*,w)
	; for-each
	(par-for-each
		(lambda (word)
			(compute-pair-wildcard-counts comp-obj word)
			(trace-msg-cnt "Wildcard-count did ")
		)
		all-singletons
	)
	(trace-elapsed)
	(trace-msg "Done with wild-card counts N(*,w) and N(w,*)\n")
	(display "Done with wild-card count N(*,w) and N(w,*)\n")

	; Now, compute the grand-total
	(store-atom (comp-obj 'cache-total-count))
	(trace-elapsed)
	(trace-msg "Done computing N(*,*), start computing log P(*,w)\n")
	(display "Done computing N(*,*), start computing log P(*,w)\n")

	; Compute the left and right wildcard logli's
	(batch-all-pair-wildcard-logli OBJ all-singletons)
	(trace-elapsed)
	(trace-msg "Done computing -log N(w,*)/N(*,*) and <-->\n")
	(display "Done computing -log N(w,*)/N(*,*) and <-->\n")

	; Enfin, the word-pair mi's
	(start-trace "Going to do individual word-pair MI\n")
	(display "Going to do individual word-pair MI\n")
	; for-each
	(par-for-each
		(lambda (word)
			(compute-pair-mi OBJ word)
			(trace-msg-cnt "Done with pair MI cnt=")
		)
		all-singletons
	)
	(trace-elapsed)
	(trace-msg "Finished with MI computations\n")
	(display "Finished with MI computations\n")
)

; ---------------------------------------------------------------------
