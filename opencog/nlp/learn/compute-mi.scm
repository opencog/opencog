;
; compute-mi.scm
;
; Compute the mutual information of language word pairs.
;
; Copyright (c) 2013, 2014, 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below are concerned with counting "things" and pairs of
; "things", and computing the mutual information of pairs. In the
; desription that follows, it will be assumed that the "things" are
; words, just to keep things simple.  However, the scripts are general,
; and are not limited to just words.
;
; It is presumed that a database of counts of word-pairs has been already
; generated; these scripts work off of those counts. Typically, the word
; pairs are obtained from parsing some language.  We say "database" here,
; instead of "atomspace", because the scripts will automatically fetch the
; needed data from the (SQL) persistence backend, as  needed.  This
; allows long-running and parallel-parsing efforts.
;
; It is assumed that all word-pair observations are stored as the
; "count" portion of the CountTruthValue on the EvaluationLink below:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "Blah"
;      ListLink
;         WordNode "some-word"
;         WordNode "other-word"
;
; The type 'WordNode is configurable; see item-type, below. The name of
; the EvaluationLink does not need to be LinkGrammarRelationshipNode,
; it can be anything, and is usually passed as the lg_rel paramter to
; most of these routines.
;
; Let N(wl,wr) denote the number of times that the word-pair (wl, wr) has
; actually been observed; that is, N("some-word", "other-word") for the
; example link above.  Properly speaking, this count is conditioned on
; the LinkGrammarRelationshipNode "Blah", so the correct notation would
; be N(wl, wr | rel)  with rel the relationship.  In what follows, the
; relationship is always assumed to be the same, and is thus dropped.
; (the relationship is passed as the lg_rel parameter everywhere).
;
; The mutual information for a word-pair is defined as follows:  Given
; two words, wl and wr, define three probabilities:
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
; These sums are computed, for a given word, by compute-pair-wildcard-counts
; below, and are computed for all words by batch-all-pair-wildcard-counts
; The resulting counts are stored as the 'count' value on the
; CountTruthValue on the EvaluationLink for structures of the form:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "Blah"
;      ListLink
;         AnyNode "left-word"
;         WordNode "bird"
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "Blah"
;      ListLink
;         WordNode "word"
;         AnyNode "right-word"
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "Blah"
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
; Similarly, N(*) can be obtained by calling `total-word-observations`.
; The count N(w) for a fixed word w can be gotten by calling
; `get-word-count-str`.
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
; The mutual information between a pair of words is defined as
;
;     MI(wl,wr) = -(H(wl,wr) - H(wl,*) - H(*,wr))
;
; This is computed by the script batch-all-pair-mi below. The value is
; stored under the key of (Predicate "*-Pair MI Key-*") as a single
; float.
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
; Define the "things" that will be pair-summed over.
; These are set as globals here, they really should be local to the
; environment; this should be fixed someday, if we ever do this for
; non-word types.

(define item-type 'WordNode)
(define item-type-str "WordNode")

; Cache the created atoms for left an right.
(define any-left (AnyNode "left-word"))
(define any-right (AnyNode "right-word"))

; ---------------------------------------------------------------------

(define-public (fetch-all-words)
"
  fetch-all-words - fetch all WordNodes from the database backend.
"
	(load-atoms-of-type item-type)
)

(define-public (get-all-words)
"
  get-all-words - return a list holding all of the observed words
  This does NOT fetch the words from the backing store.
"
	(cog-get-atoms item-type)
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

; ---------------------------------------------------------------------
; return all the ListLinks of arity two in which the word appears
(define (get-word-pairs word)
	(filter
		(lambda (lnk) (and
				(equal? 'ListLink (cog-type lnk))
				(equal? 2 (cog-arity lnk))))
		(cog-incoming-set-by-type word))
xxxxxxx
)

; The left-stars have the word in the right slot and
; have some WordNode (item-type) in the left slot.
; We must check for WordNode in this spot, as some of
; these ListLinks may have AnyNode in that slot.
(define (get-left-stars word list-links)
	(filter
		(lambda (lnk)
			(define oset (cog-outgoing-set lnk))
			(and
				(equal? item-type (cog-type (car oset)))
				(equal? word (cadr oset))))
		list-links)
)

(define (get-right-stars word list-links)
	(filter
		(lambda (lnk)
			(define oset (cog-outgoing-set lnk))
			(and
				(equal? word (car oset))
				(equal? item-type (cog-type (cadr oset))))
		list-links)
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
		(cog-incoming-set PAIR)))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Random-tree parse word-pair count access routines.
; ---------------------------------------------------------------------
;
; Given a ListLink holding a word-pair, return the corresponding
; link that holds the count for that word-pair.
(define (get-any-pair-link PAIR)
	(get-pair-link 'EvaluationLink any-pair-pred PAIR)

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
; Get the left wild-card logli for word and lg_rel.
; (that is, the wildcard is on the left side)

(define (get_left_wildcard_logli word lg_rel)
	(get-logli
		(EvaluationLink lg_rel (ListLink any-left word)))
)

; ---------------------------------------------------------------------
; Get the right wild-card logli for word and lg_rel.
; (that is, the wildcard is on the right side)

(define (get_right_wildcard_logli word lg_rel)
	(get-logli
		(EvaluationLink lg_rel (ListLink word any-right)))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Clique-based-counting word-pair access methods.
; ---------------------------------------------------------------------

; Given a ListLink holding a word-pair, return the corresponding
; link that holds the count for that word-pair.
(define (get-clique-pair-link PAIR)
	(get-pair-link 'EvaluationLink pair-pred PAIR)

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

(define (compute-pair-wildcard-counts word
	GET-PAIR GET-LEFT-WILD GET-RIGHT-WILD)

	(let* (
			; list-links are all the ListLinks in which the word appears
			(list-links (get-word-pairs word))

			; The left-stars have the word in the right slot and
			; have some WordNode (item-type) in the left slot.
			; We must check for WordNode in this spot, as some of
			; these ListLinks may have AnyNode in that slot.
			(left-stars
				(filter
					(lambda (lnk)
						(define oset (cog-outgoing-set lnk))
						(and
							(equal? item-type (cog-type (car oset)))
							(equal? word (cadr oset))
						)
					)
					list-links
				)
			)
			; The right-stars have the word in the left slot and
			; have some WordNode (item-type) in the right slot.
			(right-stars
				(filter
					(lambda (lnk)
						(define oset (cog-outgoing-set lnk))
						(and
							(equal? word (car oset))
							(equal? item-type (cog-type (cadr oset)))
						)
					)
					list-links
				)
			)

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
			(store-atom (set-count (GET-LEFT-WILD word) left-total)))

		(if (< 0 right-total)
			(store-atom (set-count (GET-RIGHT-WILD word) right-total)))
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
	GET-LEFT-WILD GET-RIGHT-WILD GET-PAIR-TOTAL ALL-WORDS)

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
	(store-atom (set-count (GET-PAIR-TOTAL) r-cnt))
	(trace-msg "Done with all-pair count\n")
)

; ---------------------------------------------------------------------
; Compute the log-liklihood for all wild-card wordpairs, for lg_rel.
; lg_rel is a link-gramar link (LinkGrammarRelationshipNode)
;
; This computes the logliklihood for all wild-card pairs of the form:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "Blah"
;      ListLink
;         WordNode "some-word"
;         AnyNode "right-word"
;
; and also for the flipped version (exchange WordNode and AnyNode)
; Here, lg_rel is (LinkGrammarRelationshipNode "Blah")
;
; The log likelihood is stored as a value on the eval
; link truth value (where logli's are always stored).
;
; The computation is performed "batch style", it assumes that all
; words in the atomspace; missing words will cause the corresponding
; pairs to be missed.

(define (batch-all-pair-wildcard-logli
	GET-LEFT-WILD GET-RIGHT-WILD GET-PAIR-TOTAL ALL-WORDS)

	; Get the word-pair grand-total
	(define pair-total (get-count (GET-PAIR-TOTAL)))

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
(define (compute-pair-mi right-word GET-PAIR GET-PAIR-TOTAL lg_rel)

	; Get the word-pair grand-total
	(define pair-total (get-count (GET-PAIR-TOTAL)))

	(let* (
			; list-links are all the ListLinks in which the word appears
			(left-stars
				(filter
					(lambda (lnk)
						(define oset (cog-outgoing-set lnk))
						(and
							(equal? 'ListLink (cog-type lnk))
							(equal? item-type (cog-type (car oset)))
							(equal? right-word (cadr oset))
						)
					)
					(cog-incoming-set right-word)
				)
			)
			; left-evs are the EvaluationLinks above the left-stars
			; That is, they have the wild-card in the left-hand slot.
			(left-evs (concatenate!
					(map! (lambda (lnk) (GET-PAIR lnk)) left-stars)))
		)
		(begin
			(for-each

				; This lambda sets the mutual information for each word-pair.
				(lambda (pair)
					(let* (
							; the left-word of the word-pair
							(left-word (gadr pair))
							(r-logli (get_right_wildcard_logli left-word lg_rel))
							(l-logli (get_left_wildcard_logli right-word lg_rel))

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
)

; ---------------------------------------------------------------------
;
; Compute the mutual information between all pairs related by lg_rel.
;
; The mutual information is defined and computed as described in the
; overview, up top. It is computed only for those pairs that are related
; by the relationship lg_rel.  The computation is done in 'batch' form;
; the required pairs are automatically fetched from SQL backend storage
; in the course of computation.  The wild-card entropies and MI values
; are written back to the database as soon as they are computed, so as
; not to be lost.  The double-nested sums are distributed over all CPU
; cores, using guile's par-for-each, and can thus be very CPU intensive.
; Running this script can take hours, or longer (days?) depending on the
; size of the dataset.  Due to the current nature of the design of the
; atomspace, this script is not very efficient; if you just need to
; count things, writing custom scripts that do NOT use the atomspace would
; almost surely be faster.  We put up with the performance overhead here
; in order to get the flexibility that the atomspace provides.
;
(define (batch-all-pair-mi
	GET-PAIR GET-LEFT-WILD GET-RIGHT-WILD GET-PAIR-TOTAL
	lg_rel)

	(define all-the-words (get-all-words))

	(trace-msg-num "Start batching, num words="
		(length all-the-words))
	(format #t "Start batching, num words=~A\n"
		(length all-the-words))

	; First, get the left and right wildcard counts.
	; (for-each
	(par-for-each
		(lambda (word)
			(compute-pair-wildcard-counts word
				GET-PAIR GET-LEFT-WILD GET-RIGHT-WILD)
			(trace-msg-cnt "Wildcard-count did ")
		)
		all-the-words
	)
	(trace-elapsed)
	(trace-msg "Done with wild-card count\n")
	(display "Done with wild-card count\n")

	; Now, get the grand-total
	(trace-msg "Going to batch-count all-pairs\n")
	(display "Going to batch-count all-pairs\n")
	(batch-all-pair-count
		 GET-LEFT-WILD GET-RIGHT-WILD GET-PAIR-TOTAL all-the-words)
	(trace-elapsed)

	; Compute the left and right wildcard logli's
	(trace-msg "Going to batch-logli wildcards\n")
	(display "Going to batch-logli wildcards\n")
	(batch-all-pair-wildcard-logli
		GET-LEFT-WILD GET-RIGHT-WILD GET-PAIR-TOTAL all-the-words)
	(trace-elapsed)

	; Enfin, the word-pair mi's
	(start-trace "Going to do individual word-pair mi\n")
	(display "Going to do individual word-pair mi\n")
	; for-each
	(par-for-each
		(lambda (word)
			(compute-pair-mi word GET-PAIR GET-PAIR-TOTAL lg_rel)
			(trace-msg-cnt "Done with pair MI cnt=")
		)
		all-the-words
	)
	(trace-msg "Finished with MI batch\n")
	(display "Finished with MI batch\n")
	(trace-elapsed)
)

; ---------------------------------------------------------------------
; Temporary handy-dandy main entry point.

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
 any-pair-pred)
	)
)

; ---------------------------------------------------------------------
; misc utilities of research interest

(define-public (get-left-word-of-pair PAIR)
"
  get-left-word-of-pair PAIR -- Given the EvaluationLink PAIR holding
  a word-pair, return the word on the left.
"
	(gadr PAIR)
)

(define-public (get-right-word-of-pair PAIR)
"
  get-right-word-of-pair PAIR -- Given the EvaluationLink PAIR holding
  a word-pair, return the word on the right.
"
	(gddr PAIR)
)

(define-public (get-all-pairs)
"
  get-all-pairs - return a list holding all of the observed word-pairs
  Caution: this can be tens of millions long!
"
	; The list of pairs is mostly just the incoming set of the ANY node.
	; However, this does include some junk, sooo ... hey, both left and
	; right better be words.
	(filter!
		(lambda (pair)
			(and
				(equal? 'WordNode (cog-type (get-left-word-of-pair pair)))
				(equal? 'WordNode (cog-type (get-right-word-of-pair pair)))))
		(cog-incoming-by-type any-pair-pred 'EvaluationLink))
)

(define-public (total-word-observations)
"
  total-word-observations -- return a total of the number of times
  any/all words were observed.  That is, compute and return N(*),
  as defined above, and in the diary.  This does NOT work from a
  cached value.  Also, this does NOT fetch atoms from the database!
"
   (get-total-atom-count (get-all-words))
)

(define-public (total-pair-observations)
"
  total-pair-observations -- return a total of the number of times
  any/all word-pairs were observed. That is, return N(*,*) as defined
  above, and in the diary.
"
	; Just get the previously computed amount.
	(get-count
		(EvaluationLink any-pair-pred
			(ListLink
				(AnyNode "left-word")
				(AnyNode "right-word"))))
)

(define-public (get-left-count-str WORD-STR)
"
  get-left-count-str WORD-STR
  Return the number of times that WORD-STR occurs in the left side
  of the \"ANY\" relationship. That is, return N(w, *), as defined above,
  and in the diary.  Here, w is WORD-STR, assumed to be a string.
"
	;; the wildcard is on the right.
	(get-count (get-any-right-wildcard (WordNode WORD-STR)))
)

(define-public (get-right-count-str WORD-STR)
"
  get-right-count-str WORD-STR
  Return the number of times that WORD-STR occurs in the right side
  of the \"ANY\" relationship. That is, return N(*, w), as defined above,
  and in the diary.  Here, w is WORD-STR, assumed to be a string.
"
	;; the wildcard is on the left.
	(get-count (get-any-left-wildcard (WordNode WORD-STR)))
)

(define-public (get-word-count-str WORD-STR)
"
  get-word-count-str WORD-STR
  Return the number of times that WORD-STR has ben observed. That is,
  return N(w) as defined above, or in the diary. Here, w is WORD-STR,
  assumed to be a string.
"
	(get-count (WordNode WORD-STR))
)

(define-public (get-total-cond-prob ALL-PAIRS)
"
  get-total-cond-prob ALL-PAIRS- return the total conditional
  probability of seeing the all word-pairs.  That is, return the
  sum over left and right words w_l, w_r of  N(w_l, w_r) / (N(w_l) N(w_r))

  Contrast this result with that of get-total-pair-prob
"
	; Return N(w_l, w_r) / N(w_l) N(w_r)
	(define (term pair)
		(/ (get-count pair)
			(* (get-count (get-left-word-of-pair pair))
				(get-count (get-right-word-of-pair pair)))))

	; textbook tail-recursive solution.
	(define (term-sum lst cnt)
		(if (null? lst) cnt
			(term-sum (cdr lst) (+ cnt (term (car lst))))))

	(term-sum ALL-PAIRS 0)
)

(define-public (get-total-pair-prob ALL-PAIRS)
"
  get-total-pair-prob - return the total pair-wise conditional
  probability of seeing a word-pair.  That is, return the sum over
  left and right words w_l, w_r of
      N(w_l, w_r) / (N(w_l, *) N(*, w_r))

  Contrast this result with that of get-total-cond-prob
"

	; Return N(w_l, w_r) / N(w_l) N(w_r)
	(define (term pair)
		(/ (get-count pair)
			(* (get-count (get-left-word-of-pair pair))
				(get-count (get-right-word-of-pair pair)))))

	; textbook tail-recursive solution.
	(define (term-sum lst cnt)
		(if (null? lst) cnt
			(term-sum (cdr lst) (+ cnt (term (car lst))))))

	(term-sum ALL-PAIRS 0)
)

; ---------------------------------------------------------------------
; misc hand debug stuff
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
