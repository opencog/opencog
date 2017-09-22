;
; cset-class.scm
;
; Merge connector sets.
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------

;
; A grammatical class is represented as
;
;     MemberLink
;         WordNode "wordy"      ; the word itself
;         WordClassNode "noun"  ; the grammatical class of the word.
;
; Word classes have a designated grammatical behavior, using Sections,
; behaving just like the pseudo-connectors on single words. Thus, either
; a WordNode or a WordClassNode can appear in a Connector link, as
; shown below.
;
;     Section
;         WordClassNode "noun"
;         ConnectorSeq
;             Connector
;                WordClassNode "verb" ; or possibly a WordNode
;                LgConnDirNode "+"
;             Connector
;                ....
;
;
; Disjunct merging
; ----------------
; Disjunct merging can be done in one of two ways.
;
; Strict disjunct merging
; -----------------------
; In strict disjunct merging, one picks a single word, and then compares
; all connecctor sequences on that word.  If two connector sequences are
; nearly identical, differing in only one location, then those two
; connectors can be merged into one. When the connectors are merged, a
; new word-class is formed to hold the two words.
;
; There are several properties of this merge style:
; f) This seems like a "strict" way to merge, because it does not allow
;    any broadening to take place.
; g) The resulting word-class does not have any sections associated with
;    it! It cannot because of the way it was constructed, but this seems
;    wrong.
;
; Connected disjunct merging
; --------------------------
; Property g) above seems wrong: word-classes should appear fully
; connected in the graph, symmetrically.  This suggests a disjunct
; merger style that maintains connectivity.
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
; This has the nice property:
; h) The total number of sections is decreasing.
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
		(if (eq? 1 cnt) mismatch-idx #f))

	; Return false if they are equal, else return the index
	(if (or (eq? SEQA SEQB) (not (eq? (cog-arity SEQA) (cog-arity SEQB))))
		 #f
		 (get-idx))
)

; ---------------------------------------------------------------
; Example usage
;
; (define pca (make-pseudo-cset-api))
; (define psa (add-dynamic-stars pca))
;
; Verify that support is correctly computed.
; (define (bogus a b) (format #t "Its ~A and ~A\n" a b))
; (define ptu (add-tuple-math psa bogus))
; (define run-n-jump (ptu 'right-stars (list (Word "run") (Word "jump"))))
; (ptu 'pair-count (car run-n-jump))
;
; (merge-ortho psa (Word "run") (Word "jump"))
