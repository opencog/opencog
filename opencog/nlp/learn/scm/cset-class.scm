;
; cset-class.scm
;
; Merge connectors into classes of connectors -  merge connector sets.
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The merging of words into word-classes proceeds in two parts. The
; first part is reviewed in `gram-class.scm` and concists on comparing
; words to see if they share similar sets of sections. If they do,
; then the words can be judged to be similar, and merged into a word
; class.  The second part, reviewed here, is to merge connector
; sequences, so that connectors become word classes, instead of just
; being individual words.
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
; Connected disjunct merging
; --------------------------
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
; typically be larger than the merged class, and so will signficantly
; broaden the grammatical reach.
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix) (opencog persist))

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
; Fetch from storage (load into RAM) all connector sequences and
; sections that are potentially mergable; i.e. that use words from
; one of the provided word-classes.
;
; CLS-LST should be a list of word-classes.
;
(define (fetch-mergable-sections CLS-LST)

	(define (fetch-connectors WORD)
		(fetch-incoming-by-type WORD 'Connector)
	)

	; Loop over all WordClassNodes
	(for-each
		(lambda (WCLS)  ; WCLS is a WordClassNode
			(for-each
				(lambda (MEMB)  ; MEMB is a MemberLink
					; The zeroth atom in the MemberLink is the WordNode.
					(fetch-connectors (cog-outgoing-atom MEMB 0)))
				(cog-incoming-by-type WCLS 'MemberLink)))
		CLS-LST)
)

; ---------------------------------------------------------------
; Example usage
;
; (define pca (make-pseudo-cset-api))
; (define psa (add-dynamic-stars pca))
;
; (define cls-lst (cog-get-atoms 'WordClassNode))
; (fetch-mergable-sections cls-lst)
