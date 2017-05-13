;
; batch-word-pair.scm
;
; Batch-compute the mutual information of pairs of nautral-language words.
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
; They are designed to run in as a batch, and may take hours to
; complete. The results are stored in the database, for future reference.
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

; ---------------------------------------------------------------------
; Random-tree parse word-pair count access routines.
;
; This implements a word-pair object, where the two words are connected
; with an LG link-type of "ANY", in an EvaluationLink.
(define (make-any-link)
	(let ()
		(define (get-left-type) 'WordNode)
		(define (get-right-type) 'WordNode)

		; Return the atom holding the count, if it exists, else
		; return nil.
		(define (get-pair PAIR)
			(cog-link 'EvaluationLink any-pair-pred PAIR))

		; Create an atom to hold the count (if it doesn't exist already).
		(define (make-pair PAIR)
			(EvaluationLink any-pair-pred PAIR))

		; Return a list of atoms hold the count.
		(define (get-pairs PAIR)
			(define pr (get-pair PAIR))
			(if (null? pr) '() (list pr)))

		; Caution: this unconditionally creates the wildcard pair!
		(define (get-left-wildcard WORD)
			(get-pair (ListLink any-left WORD)))

		; Caution: this unconditionally creates the wildcard pair!
		(define (get-right-wildcard WORD)
			(get-pair (ListLink WORD any-right)))

		(define (get-wild-wild)
			(get-pair (ListLink any-left any-right)))

	; Methods on the object
	(lambda (message . args)
		(apply (case message
				((left-type) get-left-type)
				((right-type) get-right-type)
				((item-pair) get-pair)
				((make-pair) make-pair)
				((item-pairs) get-pairs)
				((left-wildcard) get-left-wildcard)
				((right-wildcard) get-right-wildcard)
				((wild-wild) get-wild-wild)
				(else (error "Bad method call on ANY-link:" message)))
			args))))


; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Clique-based-counting word-pair access methods.
; ---------------------------------------------------------------------

; Object for getting word-pair counts, obtained from clique counting.
; The counts are stored on EvaluationLinks with the predicate
; (PredicateNode "*-Sentence Word Pair-*")
;
(define (make-clique-pair)
	(let ()
		(define (get-left-type) 'WordNode)
		(define (get-right-type) 'WordNode)

		; Return the atom holding the count, if it exists, else
		; return nil.
		(define (get-pair PAIR)
			(cog-link 'EvaluationLink pair-pred PAIR))

		; Create an atom to hold the count (if it doesn't exist already).
		(define (make-pair PAIR)
			(EvaluationLink pair-pred PAIR))

		; Return a list of atoms hold the count.
		(define (get-pairs PAIR)
			(define pr (get-pair PAIR))
			(if (null? pr) '() (list pr)))

		(define (get-left-wildcard WORD)
			(get-pair (ListLink any-left WORD)))

		(define (get-right-wildcard WORD)
			(get-pair (ListLink WORD any-right)))

		(define (get-wild-wild)
			(get-pair (ListLink any-left any-right)))

	; Methods on the object
	(lambda (message . args)
		(apply (case message
				((left-type) get-left-type)
				((right-type) get-right-type)
				((item-pair) get-pair)
				((make-pair) make-pair)
				((item-pairs) get-pairs)
				((left-wildcard) get-left-wildcard)
				((right-wildcard) get-right-wildcard)
				((wild-wild) get-wild-wild)
				(else (error "Bad method call on clique-pair:" message)))
			args))))



; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (fetch-all-words)
"
  fetch-all-words - fetch all WordNodes from the database backend.
"
	(define start-time (current-time))
	(load-atoms-of-type 'WordNode)
	(format #t "Elapsed time to load words: ~A secs\n"
		(- (current-time) start-time))
)

(define-public (fetch-any-pairs)
"
  fetch-any-pairs -- fetch all counts for link-grammar ANY links
  from the database.
"
	(define start-time (current-time))
	(fetch-incoming-set any-pair-pred)
	(format #t "Elapsed time to ANY-link pairs: ~A secs\n"
		(- (current-time) start-time))
)

(define-public (fetch-clique-pairs)
"
  fetch-clique-pairs -- fetch all counts for clique-pairs from the
  database.
"
	(define start-time (current-time))
	(fetch-incoming-set pair-pred)
	(format #t "Elapsed time to load clique pairs: ~A secs\n"
		(- (current-time) start-time))

	(set! start-time (current-time))
	(fetch-incoming-set pair-dist)
	(format #t "Elapsed time to load clique-pair distances: ~A secs\n"
		(- (current-time) start-time))
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
	(start-trace "Begin loading words\n")
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
		(make-compute-count
			(make-pair-wild (make-pair-count-get-set (make-any-link))))
		(get-all-words))
)

(define-public (batch-clique-pairs)
	(init-trace "/tmp/progress")

	; Make sure all words are in the atomspace
	(start-trace "Begin loading words\n")
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
		(make-compute-count
			(make-pair-wild (make-pair-count-get-set (make-clique-pair))))
		(get-all-words))
)

; ---------------------------------------------------------------------
; misc unit-test-by-hand stuff
;
; (use-modules (opencog) (opencog persist) (opencog persist-sql))
; (use-modules (opencog nlp) (opencog nlp learn))
; (sql-open "postgres:///en_snapshot?user=linas
; (use-modules (opencog cogserver))
; (start-cogserver "opencog2.conf")
; (fetch-all-words)
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
