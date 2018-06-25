;
; cset-class.scm
;
; Merge connectors into classes of connectors -  merge connector sets.
;
; Copyright (c) 2017, 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The merging of words into word-classes proceeds in two parts. The
; first part is reviewed in `gram-class.scm` and proceeds by comparing
; words to see if they share similar sets of sections. If they do,
; then the words can be judged to be similar, and merged into a word
; class.  The second part, reviewed here, is to merge connector
; sequences, so that connectors are made from word classes, instead of
; of individual words.
;
; Before any merging, a single section on a single word has the
; general form
;
;     Section
;         WordNode "foo"
;         ConnectorSeq
;             Connector
;                WordNode "bar"
;                LgConnDirNode "+"
;             Connector
;                ....
;
; The goal of connector-set merging (aka disjunct-merging) is to
; replace the WordNode's in a Connector by WordClassNode's, and
; thence merge two similar ConnectorSeq's into one.  Of course,
; such a merger can be done only if it "makes sense", and preserves
; the overall grammatical structure.
;
; Recall that a grammatical class is represented as
;
;     MemberLink
;         WordNode "wordy"      ; the word itself
;         WordClassNode "noun"  ; the grammatical class of the word.
;
;
; Single-difference merging
; -------------------------
; In single-difference merging, one compares all connector sequences
; on some given word.  If two connector sequences are nearly identical,
; differing in only one location, then those two connectors can be
; merged into one. When the connectors are merged, a new word-class is
; formed to hold the two words.
;
; There are several properties of this merge style:
; a) The total number of sections on a word decreases as a result
;    of the merge.  The total atom count probably decreases slightly.
; b) This seems like a "strict" or very conservative way to merge,
;    because it does not create any broadening of the allowed
;    connectors on a word. That is, the grammatical structure on the
;    word is not altered by this merge.
; c) The resulting word-class does not have any sections associated with
;    it! It cannot because of the way it was constructed, but this seems
;    wrong. Its a kind-of dead-end word-class.
;
; Connected merging
; -----------------
; Property c) above seems wrong: word-classes should appear fully
; connected in the graph, symmetrically.  This suggests a merger
; policy that can maintain graph connectivity.
;
; As above, given a single word, one scans the sections, looking
; for sections that differ in only one location. As before, the words
; that appear at this variable location are tossed into a set. However,
; this time, a search is made to see if this set overlaps, or is
; a subset of an existing grammatical class. If so, then the counts
; on all of these sections are totalled, a new disjunct is created,
; using the grammatical class in the connector, and the individual
; sections are discarded. (If there are multiple grammatical classes
; that might be appropriate, then a cosine similarity could be used
; to pick between them.)
;
; This merge style overcomes the objection in c), in that the merged
; class is already participating in many sections. Note, though, it
; is no longer conservative: the existing grammatical class will
; typically be larger than the merged class, and so will significantly
; broaden the grammatical reach.
;
; Generous merging
; ----------------
; Another possibility is a generous merge, where, whenever a word
; appears in a ConnectorSeq, and that word is also in a WordClass,
; then the word is immediately replaced by the WordClass it is in.
; This has the properties:
;
; d) The grammatical usage of that particular connector sequence
;    is immediately broadened to the new WordClass.
; e) The procedure is questionable if the word belongs to more
;    than one WordClass.
; f) This algorithm is O(N) in the number N of sections, as opposed
;    to O(N^2) or worse for the others.  That is, one need only loop
;    once over all the sections, and replace words by word-classes.
;    There is no need to compare sections pair-wise. (with the de
;    facto property that the vast majority of such compares will
;    be rejected.)
;
; Property e) is what contrasts this to the connected-merge strategy
; above: in the connected-merge, a search is made for at least two
; words belonging to the same class, to confirm (disambiguate) the
; class membership.
;
;
; Stingy merging
; --------------
; Some of the word-merging results in word classes that are too broad.
; Can some form of stingy connector merging be used to narrow down those
; overly-broad classes?  So, for example, if a `single-difference merge`
; as described above differs too much from an existing word-class, then
; perhaps the existing word class was formed too broadly?
;
; Two problems here: (1) its not clear if the above hypothesis is true
; (viz, that the original mere was too broad), and (2) there is no
; effective mechanism to un-merge.
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog sheaf) (opencog persist))

; ---------------------------------------------------------------
; Return a list of all words that belong to some grammatical class.

(define (get-classified-words)
	; Trace the MemberLink
	(define (memb CLS) (map gar (cog-incoming-by-type CLS 'MemberLink)))
	; Discard everything that is not a word.
	(define (wmemb CLS)
		(filter (lambda (w) (eq? 'WordNode (cog-type w))) (memb CLS)))
	; concqatenate them all together.
	(fold (lambda (CLS lst) (append! (memb CLS) lst)) '()
		(cog-get-atoms 'WordClassNode))
)

; ---------------------------------------------------------------
;
; Given a word or word-class, return a list of all sections attached to
; that word or word-class that are potentially mergable; that is, have
; a disjunct with a connector that belongs to an existing WordClass.
; The goal is to trim the list of sections to something smaller.
;
; XXX FIXME this might be pointless and useless?
(define (get-all-sections-in-classes WCL)

	; Return not-#f if the connector is in any class.
	(define (connector-in-any-class? CTR)
		(define wrd-of-ctr (gar CTR)) ; Word of the connector
		(find (lambda (MEMB)
				(eq? 'WordClassNode (cog-type (gdr MEMB))))
			(cog-incoming-by-type wrd-of-ctr 'MemberLink)))

	; Return not-#f if section SEC has connectors that
	; are in some WordClass, any WordClass
	(define (classifiable-section? SEC)
		; list of connectors in the section
		(define con-seq (cog-outgoing-set (gdr SEC)))
		(find connector-in-any-class? con-seq))

	; Return list of all sections that have connectors that are
	; in some (any) WordClass.
	(filter classifiable-section? (cog-incoming-by-type WCL 'Section))
)

; ---------------------------------------------------------------
;
; Given a word or a word-class, return a list of all sections that
; have disjuncts with N connectors.
;
(define (get-sections-by-size WCL SIZ)

	(define sects (get-all-sections-in-classes WCL))

	; Return not-#f if section SEC has SIZ connectors
	(define (size-section? SEC)
		(eq? SIZ (length (cog-outgoing-set (gdr SEC)))))

	; Return list of all sections that are of the given size
	(filter size-section? sects)
)

; ---------------------------------------------------------------

(define-public (in-gram-class? WORD GCLS)
"
  in-gram-class? WORD GRAM-CLASS - is the WORD a member of the
  grammatical class CRAM-CLASS? Returns either #t or #f.
"
	(define memlnk (cog-link 'MemberLink WORD GCLS))
	(if (null? memlnk) #f #t)
)

; ---------------------------------------------------------------
; Compare two ConnectorSeq links, to see if they are the same,
; differing in only one location.  If this is the case, return
; the index offset to the location that is different. The index
; is zero-based. If they are not matchable, return #f.

(define (connector-seq-compare SEQA SEQB)
	; Get the index of the difference. Return #f if there are two
	; or more differences. If both are equal, it returns the length.
	; Could not figure out how to implement this without using set!
	(define (get-idx)
		(define mismatch-idx #f)
		(define cnt 0)
		(pair-fold
			(lambda (subseq-a subseq-b idx)
				; (format #t "duude ~A and ~A and ifx=~A\n" subseq-a subseq-b idx)
				(if (not (equal? (car subseq-a) (car subseq-b)))
					(begin (set! mismatch-idx idx)
						(set! cnt (+ cnt 1))))
				(+ idx 1))
			0 (cog-outgoing-set SEQA) (cog-outgoing-set SEQB))

		; Only one difference allowed.
		(if (eq? 1 cnt) mismatch-idx #f))

	; Return false if they are equal, else return the index
	(if (or (eq? SEQA SEQB) (not (eq? (cog-arity SEQA) (cog-arity SEQB))))
		 #f
		 (get-idx))
)

; ---------------------------------------------------------------
; Fetch from storage (load into RAM) all words that appear as members
; of one of the provided word-classes. Return the list of words.
;
; CLS-LST should be a list of word-classes.
;
; In most cases, this is not strictly necessary, as the usual case is
; that all words and word-classes are already in RAM.
;
(define (fetch-class-words CLS-LST)

	; Return a list without duplicates.
	(delete-dup-atoms
		(concatenate!
			; Loop over all WordClassNodes
			(map
				; This lambda returns a list of words.
				(lambda (CLS)  ; CLS is a WordClasNode
					(fetch-incoming-by-type CLS 'MemberLink)
					; map converts list of MemberLinks into list of words.
					(map
						; MEMB is a MemberLink; the zeroth atom in
						; the MemberLink is the WordNode.
						(lambda (MEMB) (cog-outgoing-atom MEMB 0))
						(cog-incoming-by-type CLS 'MemberLink)))
				CLS-LST)))
)

; ---------------------------------------------------------------
; Fetch from storage (load into RAM) all connector sequences and
; sections that are potentially mergeable; i.e. that use words from
; one of the provided word-classes. This returns a list of all the
; sections that contain a connector using a word from one of the
; word-classes.
;
; CLS-LST should be a list of word-classes.
;
(define (fetch-mergable-sections CLS-LST)

	; Loop over all WordClassNodes
	(delete-dup-atoms
		(map fetch-endpoint-sections
			(fetch-class-words CLS-LST)))
)

; ---------------------------------------------------------------
; Example usage
;
; (define cls-lst (cog-get-atoms 'WordClassNode))
; (fetch-mergable-sections cls-lst)
;
; (define wrd (Word "chance"))
; (define secs (cog-incoming-by-type wrd 'Section))
; (define maybe (get-all-sections-in-classes wrd))
