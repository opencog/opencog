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
; The scripts below are concerned with counting "items" and pairs of
; "items", and computing the mutual information of the pairs. In the
; current usage, the "items" are always WordNodes, and the pairs are
; word-pairs obtained from lingistic analysis.  However, these scripts
; are general, and work for any kind of pairs, not just words.
;
; It is presumed that a database of counts of pairs has been already
; generated; these scripts work off of those counts. Typically, the
; pairs are obtained from parsing some language.  We say "database" here,
; instead of "atomspace", because the scripts will automatically fetch the
; needed data from the (SQL) persistence backend, as  needed.  This
; allows long-running and parallel-parsing efforts.
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
; GET-PAIR function returns the atom holding the count.
;
; Let N(wl,wr) denote the number of times that the pair (wl, wr) has
; actually been observed; that is, N("some-word", "other-word") for the
; example above.  Properly speaking, this count is conditioned on the
; LinkGrammarRelationshipNode "ANY", so the correct notation would be
; N(rel, wl, wr) with `rel` the relationship.  In what follows, the
; relationship is always assumed to be the same, and is thus dropped.
; (the relationship is provided throught the GET-PAIR functiion).
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
; The pair-counting infrastructure here is generic, because there
; are several different ways of counting pairs. The counting is done
; in `link-pipeline.scm`.  One can: count pairs using an
; all-possible-pair approach; this is called "clique counting".
; One can count pairs by parsing with a random grammar, this is called
; "link-type-any" counting. Finally, one can do either of the above,
; and track the lengths of the links, and restrict to those links
; less than a given length.  In this last case, the lengths are
; stored in ExecutionLinks, not EvaluationLinks, and a SchemaNode
; instead of a PredicateNode is used to identify where the counts
; are held. The code below should work fine with each of these types.
;
; That's all there's to this. The batch-all-pair-mi is the main entry
; point; its given at the bottom.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (ice-9 threads))
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
; Return all of the ListLinks of arity two in which the ITEM appears
(define (get-item-pairs ITEM)
	(filter
		(lambda (lnk) (equal? 2 (cog-arity lnk)))
		(cog-incoming-by-type ITEM 'ListLink))
)

; The left-stars have the item in the right slot and have some other
; item (of ITEM-TYPE) in the left slot. We must check for ITEM-TYPE
; in this spot, as some of these ListLinks may have AnyNode (or
; something else) in that slot.
(define (get-left-stars word list-of-pairs ITEM-TYPE)
	(filter
		(lambda (lnk)
			(define oset (cog-outgoing-set lnk))
			(and
				(equal? ITEM-TYPE (cog-type (car oset)))
				(equal? word (cadr oset))))
		list-of-pairs)
)

(define (get-right-stars word list-of-pairs ITEM-TYPE)
	(filter
		(lambda (lnk)
			(define oset (cog-outgoing-set lnk))
			(and
				(equal? word (car oset))
				(equal? ITEM-TYPE (cog-type (cadr oset)))))
		list-of-pairs)
)

; ---------------------------------------------------------------------
; Count the total number of times that the atoms in the atom-list have
; been observed.  The observation-count for a single atom is stored in
; the 'count' value of its CountTruthValue. This routine just fetches
; those, and adds them up.
;
; The returned value is the total count.

(define-public (get-total-atom-count atom-list)
	;
	; A proceedural loop.
	;	(let ((cnt 0))
	;		(define (inc atom) (set! cnt (+ cnt (get-count atom))))
	;		(for-each inc atom-list)
	;		cnt
	;	)

	; textbook tail-recursive solution.
	(define (hlpr lst cnt)
		(if (null? lst) cnt
			(hlpr (cdr lst) (+ cnt (get-count (car lst))))))

	(hlpr atom-list 0)
)

; ---------------------------------------------------------------------
; Compute log liklihood of having observed a given atom.
;
; The liklihood and its log-base-2 will be stored under the key
; (Predicate "*-FrequencyKey-*"), with the first number being the
; frequency, which is just the atom's count value, dividing by the
; total number of times the atom has been observed.  The log liklihood
; is -log_2(frequency), and is stored as a convenience.
;
; This returns the atom that was provided, but now with the logli set.

(define (compute-atom-logli atom total)
	(set-freq atom (/ (get-count atom) total))
)

; ---------------------------------------------------------------------
; Compute the occurance logliklihoods for a list of atoms.
;
; This sums up the occurance-count over the entire list of atoms,
; and uses that as the normalization for the probability frequency
; for the individual atoms in the list. It then computes the log_2
; likelihood for each atom in the list, based on the total.
;
; As usual, the raw counts are obtained from the 'count' slot on a
; CountTruthValue, and the logli is stored as a value on the atom.
;
; This returns the atom-list, but now with the logli's set.

(define (compute-all-logli atom-list)
	(let ((total (get-total-atom-count atom-list)))
		(map
			(lambda (atom) (compute-atom-logli atom total))
			atom-list
		)
	)
)

; ---------------------------------------------------------------------
; Compute the occurance logliklihoods for all words.
;
; Load all word-nodes into the atomspace from SQL storage, if they
; are not already present.  This also loads the associated values.
;
; This returns the list of all word-nodes, with the logli's set.

(define (compute-all-word-freqs)
	(begin
		; Make sure that all word-nodes are in the atom table.
		(fetch-all-words)
		(compute-all-logli (get-all-words))
	)
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

(define (compute-pair-wildcard-counts ITEM
	GET-PAIR GET-LEFT-WILD GET-RIGHT-WILD ITEM-TYPE)

	(let* (
			; list-links are all the ListLinks in which the ITEM appears
			(list-links (get-item-pairs ITEM))

			; The left-stars have the item in the right slot and
			; have some other ITEM-TYPE in the left slot. We must
			; check for ITEM-TYPE in this spot, as some of these
			; ListLinks may have AnyNode in that slot.
			(left-stars (get-left-stars ITEM list-links ITEM-TYPE))

			; The right-stars have the item in the left slot and
			; have some other ITEM-TYPE in the right slot.
			(right-stars (get-right-stars ITEM list-links ITEM-TYPE))

			; left-evs are the EvaluationLinks above the left-stars
			; That is, they have the wild-card in the left-hand slot.
			(left-evs (concatenate!
					(map!  (lambda (lnk) (GET-PAIR lnk)) left-stars)))

			(right-evs (concatenate!
					(map!  (lambda (lnk) (GET-PAIR lnk)) right-stars)))

			; The total occurance counts
			(left-total (get-total-atom-count left-evs))
			(right-total (get-total-atom-count right-evs))
		)

		(if (< 0 left-total)
			(store-atom (set-count (GET-LEFT-WILD ITEM) left-total)))

		(if (< 0 right-total)
			(store-atom (set-count (GET-RIGHT-WILD ITEM) right-total)))
	)
)

; ---------------------------------------------------------------------
; Compute the total number of observed word-pairs, using the left
; and right wild-card count access methods.  The resulting total count
; is stored using the STORE-PAIR-TOTAL function.
;
; The computation is performed batch-style. This function assumes that
; all word-pairs are already loaded in the atom table; it will get
; incorrect counts if this is not the case.
;
(define (batch-all-pair-count
	GET-LEFT-WILD GET-RIGHT-WILD GET-WILD-WILD ALL-WORDS)

	(define l-cnt 0)
	(define r-cnt 0)

	(start-trace "Start all-pair-count\n")
	; Now, loop over all words, totalling up the counts.
	(for-each
		(lambda (word)
			(set! l-cnt (+ l-cnt (get-count (GET-LEFT-WILD word))))
			(set! r-cnt (+ r-cnt (get-count (GET-RIGHT-WILD word))))
		)
		ALL-WORDS)

	; The left and right counts should be equal!
	(if (not (eqv? l-cnt r-cnt))
		(throw 'bad-summation 'batch-all-pair-count
			(format #f "Error: word-pair-counts unequal: ~A ~A\n" l-cnt r-cnt))
	)

	; Create and save the grand-total count.
	(store-atom (set-count (GET-WILD-WILD) r-cnt))
	(trace-msg "Done with all-pair count\n")
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

(define (batch-all-pair-wildcard-logli
	GET-LEFT-WILD GET-RIGHT-WILD GET-WILD-WILD ALL-WORDS)

	; Get the word-pair grand-total
	(define pair-total (get-count (GET-WILD-WILD)))

	; For each wild-card pair associated with the word,
	; obtain the log likelihood.
	(for-each
		(lambda (word)
			(let ((lefty (GET-LEFT-WILD word))
					(righty (GET-RIGHT-WILD word)))

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
(define (compute-pair-mi right-word GET-PAIR
	GET-LEFT-WILD GET-RIGHT-WILD GET-WILD-WILD ITEM-TYPE)

	; Get the word-pair grand-total
	(define pair-total (get-count (GET-WILD-WILD)))

	(let* (
			; left-stars are all the ListLinks in which the right-word
			; appears on the right (and anything on the left)
			(left-stars (get-left-stars (get-item-pairs right-word) ITEM-TYPE))

			; left-evs are the EvaluationLinks above the left-stars
			; That is, they have the wild-card in the left-hand slot.
			(left-evs (concatenate!
					(map! (lambda (lnk) (GET-PAIR lnk)) left-stars)))

			(l-logli (get-logli (GET-LEFT-WILD right-word)))
		)
		(for-each

			; This lambda sets the mutual information for each word-pair.
			(lambda (pair)
				(let* (
						; the left-word of the word-pair
						(left-word (gadr pair))

						(r-logli (get-logli (GET-RIGHT-WILD left-word)))

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
				)
			)
			left-evs
		)
	)
)

; ---------------------------------------------------------------------
;
; Compute the mutual information between all pairs.
;
; The mutual information between pairs is described in the overview,
; up top of this file. The access to the pairs is governed by the
; the access functions provided as arguments.
;
; The `all-singletons` argument should be a list of all items over
; which the pairs will be computed. Every item in this list should
; be of ITEM-TYPE.
;
; The GET-PAIR argument should be a function that returns all atoms
; that hold counts for a pair (ListLink left-item right-item). The
; algorithm uses a doubley-nested loop to walk over all pairs, in
; a sparse-matrix fashion: The outer loop is over all all items,
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
(define (batch-all-pair-mi GET-PAIR
	GET-LEFT-WILD GET-RIGHT-WILD GET-WILD-WILD ITEM-TYPE all-singletons)

	(trace-msg-num "Start batching, num words="
		(length all-singletons))
	(format #t "Start batching, num words=~A\n"
		(length all-singletons))

	; First, get the left and right wildcard counts.
	; (for-each
	(par-for-each
		(lambda (word)
			(compute-pair-wildcard-counts word
				GET-PAIR GET-LEFT-WILD GET-RIGHT-WILD ITEM-TYPE)
			(trace-msg-cnt "Wildcard-count did ")
		)
		all-singletons
	)
	(trace-elapsed)
	(trace-msg "Done with wild-card count\n")
	(display "Done with wild-card count\n")

	; Now, get the grand-total
	(trace-msg "Going to batch-count all-pairs\n")
	(display "Going to batch-count all-pairs\n")
	(batch-all-pair-count
		 GET-LEFT-WILD GET-RIGHT-WILD GET-WILD-WILD all-singletons)
	(trace-elapsed)

	; Compute the left and right wildcard logli's
	(trace-msg "Going to batch-logli wildcards\n")
	(display "Going to batch-logli wildcards\n")
	(batch-all-pair-wildcard-logli
		GET-LEFT-WILD GET-RIGHT-WILD GET-WILD-WILD all-singletons)
	(trace-elapsed)

	; Enfin, the word-pair mi's
	(start-trace "Going to do individual word-pair mi\n")
	(display "Going to do individual word-pair mi\n")
	; for-each
	(par-for-each
		(lambda (word)
			(compute-pair-mi word GET-PAIR
				GET-LEFT-WILD GET-RIGHT-WILD GET-WILD-WILD ITEM-TYPE)
			(trace-msg-cnt "Done with pair MI cnt=")
		)
		all-singletons
	)
	(trace-msg "Finished with MI batch\n")
	(display "Finished with MI batch\n")
	(trace-elapsed)
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define any-left (AnyNode "left-word"))
(define any-right (AnyNode "right-word"))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; get-pair-link returns a list of atoms of type LNK-TYPE that contains
; the given PRED and PAIR.
;
; PAIR is usually a ListLink of word-pairs.
; PRED is usually (PredicateNode "*-Sentence Word Pair-*")
;                 or (SchemaNode "*-Pair Distance-*")
;                 or (LinkGrammarRelationshopNode "ANY")
; LNK-TYPE is usually EvaluationLink or ExecutationLink
;
; PAIR is the atom (ListLink) we are given.  We want to find the
; LNK-TYPE that contains it.  That LNK-TYPE should have
; PRED as its predicate type, and the ListLink in the expected
; location. That is, given ListLink, we are looking for
;
; The result of this search is either #f or the EvaluationLink

(define (get-pair-link LNK-TYPE PRED PAIR)

	(filter!
		(lambda (evl)
			(define oset (cog-outgoing-set evl))
			(and
				(equal? LNK-TYPE (cog-type evl))
				(equal? PRED (car oset))
				(equal? PAIR (cadr oset))))
		(cog-incoming-set PAIR))
)

; ---------------------------------------------------------------------
; Random-tree parse word-pair count access routines.
;
; Given a ListLink holding a word-pair, return the corresponding
; link that holds the count for that word-pair.
;
(define (get-any-pair-link PAIR)
	(get-pair-link 'EvaluationLink any-pair-pred PAIR)
)

; Get the atom that holds the left wild-card count for `word`,
; for the LG link type "ANY". (the wildcard is on the left side)

(define (get-any-left-wildcard word)
	(EvaluationLink any-pair-pred (ListLink any-left word))
)

(define (get-any-right-wildcard word)
	(EvaluationLink any-pair-pred (ListLink word any-right))
)

; Get the atom that holds the total word-pair count.
; This has wild-cards on both the left and right.

(define (get-any-pair)
	(EvaluationLink any-pair-pred (ListLink any-left any-right))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Clique-based-counting word-pair access methods.
; ---------------------------------------------------------------------

; Given a ListLink holding a word-pair, return the corresponding
; link that holds the count for that word-pair.
;
(define (get-clique-pair-link PAIR)
	(get-pair-link 'EvaluationLink pair-pred PAIR)
)

; Get the atom that holds the left wild-card count for `word`,
; for the clique-based counts. (the wildcard is on the left side)

(define (get-clique-left-wildcard word)
	(EvaluationLink pair-pred (ListLink any-left word))
)

(define (get-clique-right-wildcard word)
	(EvaluationLink pair-pred (ListLink word any-right))
)

(define (get-clique-pair)
	(EvaluationLink pair-pred (ListLink any-left any-right))
)

; ---------------------------------------------------------------------

(define-public (get-all-words)
"
  get-all-words - return a list holding all of the observed words
  This does NOT fetch the words from the backing store.
"
	(cog-get-atoms 'WordNode)
)

(define-public (fetch-all-words)
"
  fetch-all-words - fetch all WordNodes from the database backend.
"
	(load-atoms-of-type 'WordNode)
)

(define-public (fetch-any-pairs)
"
  fetch-any-pairs -- fetch all counts for link-grammar ANY links
  from the database.
"
	(fetch-incoming-set any-pair-pred)
)

(define-public (fetch-clique-pairs)
"
  fetch-clique-pairs -- fetch all counts for clique-pairs from the
  database.
"
	(fetch-incoming-set pair-pred)
	(fetch-incoming-set pair-dist)
)

(define-public (fetch-words-once)
	(define done #f)
	(if (not done) (begin (fetch-all-words) (set! done #t)))
)

; ---------------------------------------------------------------------
; Handy-dandy main entry points.

(define-public (batch-all-pairs)
	(begin
		(init-trace "/tmp/progress")

		; Make sure all words are in the atomspace
		(fetch-all-words)
		(trace-msg "Done loading words, now loading pairs")
		(display "Done loading words, now loading pairs")

		; Make sure all word-pairs are in the atomspace.
		(fetch-any-pairs)
		(trace-msg "Finished loading word-pairs\n")
		(display "Finished loading word-pairs\n")

		(batch-all-pair-mi
			get-any-pair-link
			get-any-left-wildcard
			get-any-right-wildcard
			get-any-pair
			'WordNode
			(get-all-words))
	)
)

(define-public (batch-all-pairs)
	(begin
		(init-trace "/tmp/progress")

		; Make sure all words are in the atomspace
		(fetch-all-words)
		(trace-msg "Done loading words, now loading pairs")
		(display "Done loading words, now loading pairs")

		; Make sure all word-pairs are in the atomspace.
		(fetch-any-pairs)
		(trace-msg "Finished loading word-pairs\n")
		(display "Finished loading word-pairs\n")

		(batch-all-pair-mi
			get-any-pair-link
			get-any-left-wildcard
			get-any-right-wildcard
			get-any-pair
			'WordNode
			(get-all-words))
	)
)

; ---------------------------------------------------------------------
; misc unit-test-by-hand stuff
;
; (define x (WordNode "famille"))
; (define y (LinkGrammarRelationshipNode "ANY"))
; (fetch-and-compute-pair-wildcard-counts x y)
;
; (fetch-all-words)
; (define wc (cog-count-atoms 'WordNode))
; (length (cog-get-atoms 'WordNode))
; (define wc (get-total-atom-count (cog-get-atoms 'WordNode)))
;
; (compute-word-prob x wc)
;
; select count(*) from  atoms where type = 77;
; 16785 in fr
; 19781 in lt
;
; select * from atoms where name='famille';
; uuid is 2908473
; select * from atoms where outgoing @> ARRAY[2908473];
; select * from atoms where outgoing @> ARRAY[cast(2908473 as bigint)];
;
; 43464154
; duuude left-star handle is
; 43464157duuude good by
;
; (define wtfl  (EvaluationLink  (LinkGrammarRelationshipNode "ANY")
;   (ListLink (AnyNode "left-word") (WordNode "famille"))))
;
; (define wtfr  (EvaluationLink  (LinkGrammarRelationshipNode "ANY")
;     (ListLink (WordNode "famille") (AnyNode "right-word"))))
;
;
; anynode is type 105
;  select * from atoms where type=105;
; uuid is 43464152
;         43464155
; select count(*) from atoms where outgoing @> ARRAY[cast(43464152 as bigint)];
; returns the number of word-pairs which we've wild-carded.
; select * from atoms where outgoing = ARRAY[cast(43464152 as bigint), cast(43464155 as bigint)];
;
; How many atoms?
; 16785  words in fr
;
; How many pairs ??
; LinkGrammarRelationshipNode is 87
; ANY is uuid 199
; select count(*) from atoms where outgoing @> ARRAY[cast(199 as bigint)];
; There are 177960 french pairs.
; This should require  2x atoms per pair (eval, list)
; viz 178K x 2 = 360K atom loads.
; OK, that's good.
