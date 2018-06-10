;
; pseudo-csets.scm
;
; Compute the cosine-similarity between two pseudo-connector-sets.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute the cosine-similarity (and other similarity
; measures) between pseudo-connector-set vectors.
;
; An example connector-set, for the word "playing", illustrating
; that it can connect to the word "level" on the left, and "field"
; on the right, is shown below. (Please note that this is for
; illustrating the atomese structure, and not for illustrating actual
; English grammar; its a bad grammatical example, although the MST
; parser does find things like this.)
;
;    (Section
;       (WordNode "playing")
;       (ConnectorSeq
;          (Connector
;             (WordNode "level")
;             (LgConnDirNode "-"))
;          (Connector
;             (WordNode "field")
;             (LgConnDirNode "+"))))
;
; In dictionary short-hand, this would be written as
;
;     playing:  level- & field+;
;
; The `ConnectorSeq` part of this structure is refered to as the
; pseudo-disjunct, in that it resembles a normal linkgrammar
; disjunct, but has words appearing where connectors should be.
;
; Any given word may have dozens or hundreds or thousands of these
; sections. The totality of these sections, for a given, fixed word
; form a vector.  The connector seq (the disjunct) is a basis element,
; and the raw observational count on the `Section` is the magnitude
; of the vector in that basis direction.
;
; Note that these vectors are sparse: if a particular disjunct is
; missing, then the associated count is zero.  Note that the dimension
; of the vector-space is extremely high, possibly in the
; tens-of-millions, while a "typical" English word might have a few
; thousand non-zero sections.
;
; As vectors, dot-products can be taken. The most interesting of these
; is the cosine similarity between two words. This quantity indicates how
; similar two words are, grammatically-speaking. Other vector measures
; are interesting, including lp-similarity, etc.
;
; Not implemented: the Pearson R.  There is both a theoretical and a
; practical difficulty with it. The theoretical difficulty is that
; we never expect connector sets to be correlated, with a
; different mean offset!  This doesn't make sense, because when a
; disjunct is not seen, it literally isn't there; it does NOT get
; folded into a mean-value offset.  The practical difficulty is that
; computing the vector mean requires knowning the total dimensionality
; of the space, which can be gotten by computing
;   (length (cset-support get-all-words))
; but this can take hours to compute.  Its also huge: about 200K
; for my "small" dataset; millions in the large one.
;
; Also not implemented: Tanimoto metric. The formula for this gives
; an actual metric only when the vectors are bit-vectors, and we don't
; have bit-vectors, so I am not implementing this. Note, however, that
; the presence/absence of support can be viewed as a bit-vector.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))
(use-modules (opencog matrix))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (make-pseudo-cset-api)
"
  make-pseudo-cset-api -- connector-set access methods. Pseudo-
  connector sets are pairs consisting of a word on the left, and
  a pseudo-disjunct on the right. These are observed during MST parsing.
  A more detailed scription is at the top of this file.
"
	(let ((all-csets '()))

		; Get the observational count on ATOM
		(define (get-count ATOM) (cog-tv-count (cog-tv ATOM)))

		(define any-left (AnyNode "cset-word"))
		(define any-right (AnyNode "cset-disjunct"))

		(define (get-left-type) 'WordNode)
		(define (get-right-type) 'ConnectorSeq)
		(define (get-pair-type) 'Section)

		; Getting the pair is trivial: we already got it.
		(define (get-pair PAIR) PAIR)

		; Getting the count is trivial, we already got the needed pair.
		(define (get-pair-count PAIR) (get-count PAIR))

		(define (get-left-wildcard DJ)
			(ListLink any-left DJ))

		(define (get-right-wildcard WORD)
			(ListLink WORD any-right))

		(define (get-wild-wild)
			(ListLink any-left any-right))

		; Fetch (from the database) all pseudo-csets
		(define (fetch-pseudo-csets)
			(define start-time (current-time))
			; marginals are located on any-left, any-right
			(fetch-incoming-set any-left)
			(fetch-incoming-set any-right)
			(load-atoms-of-type 'Section)
			(format #t "Elapsed time to load csets: ~A secs\n"
				(- (current-time) start-time)))

		; Methods on the object
		(lambda (message . args)
			(apply (case message
				((name) (lambda () "Word-Disjunct Pairs (Connector Sets)"))
				((id)   (lambda () "cset"))
				((left-type) get-left-type)
				((right-type) get-right-type)
				((pair-type) get-pair-type)
				((pair-count) get-pair-count)
				((item-pair) get-pair)
				((make-pair) get-pair)
				((left-wildcard) get-left-wildcard)
				((right-wildcard) get-right-wildcard)
				((wild-wild) get-wild-wild)
				((fetch-pairs) fetch-pseudo-csets)
				((provides) (lambda (symb) #f))
				((filters?) (lambda () #f))
				(else (error "Bad method call on pseudo-cset:" message)))
			args)))
)

; ---------------------------------------------------------------------
