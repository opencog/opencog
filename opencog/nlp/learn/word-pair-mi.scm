;
; word-pair-mi.scm
;
; Compute the mutual information of pairs of nautral-language words.
;
; Copyright (c) 2013, 2014, 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute the Yuret-style lexical attraction between
; pairs of words.  They make use of the generic API for computing
; mutual information between ordered pairs in some relation.
; See `compute-mi.scm` for more detail about what is computed, and how.
;
; One structure, among several, in which the pair counts are held,
; is of the form
;
;     EvaluationLink
;         LinkGrammarRelationshipNode "ANY"
;         ListLink
;             WordNode "some-word"
;             WordNode "other-word"
;
; After they've been computed, the values for N(w,*) and N(*,w) can be
; fetched with the `get-left-count-str` and `get-right-count-str`
; routines, below.  The value for N(*,*) can be gotten by calling
; `total-pair-observations`.
;
; The counting done in `link-pipeline.scm` keeps track of several
; different types of pair information.  Besides the above, it also
; counts these things:
;
;     EvaluationLink
;         PredicateNode "*-Sentence Word Pair-*"
;         ListLink
;             WordNode "lefty"
;             WordNode "righty"
;
;     ExecutionLink
;         SchemaNode "*-Pair Distance-*"
;         ListLink
;             WordNode "lefty"
;             WordNode "righty"
;         NumberNode 3
;
; ---------------------------------------------------------------------
;
(use-modules (opencog))
(use-modules (opencog persist))

(define any-left (AnyNode "left-word"))
(define any-right (AnyNode "right-word"))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Random-tree parse word-pair count access routines.
;
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

(define (get-any-wild-wild)
	(EvaluationLink any-pair-pred (ListLink any-left any-right))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Clique-based-counting word-pair access methods.
; ---------------------------------------------------------------------

; Get the atom that holds the left wild-card count for `word`,
; for the clique-based counts. (the wildcard is on the left side)

(define (get-clique-left-wildcard word)
	(EvaluationLink pair-pred (ListLink any-left word))
)

(define (get-clique-right-wildcard word)
	(EvaluationLink pair-pred (ListLink word any-right))
)

(define (get-clique-wild-wild)
	(EvaluationLink pair-pred (ListLink any-left any-right))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

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

;; Call the function only once, ever.
;; The SQL loads are slow, so don't repeat them, if they are
;; not needed.
(define call-only-once
	(let ((called '()))
		(lambda (func)
			(if (not (member func called))
				(begin (func)
					(set! called (cons func called))))))
)

; ---------------------------------------------------------------------
; Handy-dandy main entry points.

(define-public (batch-any-pairs)
	(init-trace "/tmp/progress")

	; Make sure all words are in the atomspace
	(start-tace "Begin loading words\n")
	(call-only-once fetch-all-words)
	(trace-elapsed)
	(trace-msg "Done loading words, now loading any-pairs\n")
	(display "Done loading words, now loading any-pairs\n")

	; Make sure all word-pairs are in the atomspace.
	(call-only-once fetch-any-pairs)
	(trace-elapsed)
	(trace-msg "Finished loading any-word-pairs\n")
	(display "Finished loading any-word-pairs\n")

	(batch-all-pair-mi
		get-any-pair
		get-any-left-wildcard
		get-any-right-wildcard
		get-any-wild-wild
		'WordNode
		(get-all-words))
)

(define-public (batch-clique-pairs)
	(init-trace "/tmp/progress")

	; Make sure all words are in the atomspace
	(start-tace "Begin loading words\n")
	(call-only-once fetch-all-words)
	(trace-elapsed)
	(trace-msg "Done loading words, now loading clique pairs\n")
	(display "Done loading words, now loading clique pairs\n")

	; Make sure all word-pairs are in the atomspace.
	(call-only-once fetch-clique-pairs)
	(trace-elapsed)
	(trace-msg "Finished loading clique-word-pairs\n")
	(display "Finished loading clique-word-pairs\n")

	(batch-all-pair-mi
		get-clique-pair
		get-clique-left-wildcard
		get-clique-right-wildcard
		get-clique-wild-wild
		'WordNode
		(get-all-words))
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
		(call-only-once fetch-all-words)
		(compute-all-logli (get-all-words))
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
