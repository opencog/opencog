;
; gram-class.scm
;
; Merge words into grammatical categories.
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; When a pair of words are judged to be grammatically similar, they
; can be used to create a "grammatical class", containing both the
; words, and behaving as thier union/sum.  When a word is judged to
; belong to an existing grammatical-class, then some mechanism must
; be provided to add that word to the class.  This file implements
; the tools for creating and managing such classes.  It does not
; dictate how to judge when words belong to a class; this is done
; independently of the structure of the classes themselves.
;
; A grammatical class is represented as
;
;     MemberLink
;         WordNode "wordy"      ; the word itself
;         WordClassNode "noun"  ; the grammatical class of the word.
;
; Word classes have a designated grammatic behavior, using Sections,
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
; There are several ways of handling the merger of words into classes.
; Consider first merging
; The "linear" approach is to intersect the co
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix))

(define (merge-ortho LLOBJ WA WB)
"
  merge-ortho WA WB - merge WA and WB into a grammatical class.

  WA and WB should be WordNodes or WordClassNodes.
  LLOBJ is used to access counts on pairs.  Pairs are SectionLinks,
     that is, are (word,disjunct) pairs wrapped in a SectionLink.

  The merger of WA and WB are performed, using the 'orthogoanal
  merge' strategy. This is done like so. If WA and WB are both
  WordNodes, then a WordClass is created, having both WA and WB as
  members.  The counts on that word-class are the sum of the counts
  on WA and WB. Next, the counts on WA and WB are adjusted, so that
  only the orthogonal components are left (that is, the parts
  orthogonal to the sum). Next, zero-clamping is applied, so that
  any non-positive components are erased.

  If WA is a WordClassNodes, and WB is not, then WB is merged into
  WA. Currently, WB must never be a WordClass....
"
	; set-count ATOM CNT - Set the raw observational count on ATOM.
	(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 1 0 CNT)))

	; Create a new word-class.
	(define mrg-class (WordClassNode (string-concatenate
		(list (cog-name WA) " " (cog-name WB)))))

	(define (bogus a b) (format #t "Its ~A and ~A\n" a b))

	(define (merge-func WORD-PAIR)
		; The two words to merge
		(define lw (first WORD-PAIR))
		(define rw (second WORD-PAIR))

		; The counts on them or zero.
		(define lc (if (null? lw) 0 (LLOBJ 'pair-count lw)))
		(define rc (if (null? rw) 0 (LLOBJ 'pair-count rw)))
		(define cnt (+ rc lc))

		; The disjunct. Both lw and rw have the same disjunct.
		(define seq (if (null? lw) (cog-outgoing-atom rw 1)
				(cog-outgoing-atom lw 1)))

		; The merged word-class
		(define mrg (Section mrg-class seq))

		; The summed counts
		(set-count mrg cnt)

		; (store-atom mrg) ; save to SQL
	)

	; Put the two words into the new word-class.
	(MemberLink WA mrg-class)
	(MemberLink WB mrg-class)

	(let* ((ptu (add-tuple-math psa bogus))
			(mrg-basis (ptu 'right-stars (list WA WB)))
		)

		(for-each merge-func mrg-basis)
	)
)


; (define psa (add-dynamic-stars pca))
; (define (bogus a b) (format #t "Its ~A and ~A\n" a b))
; (define ptu (add-tuple-math psa bogus))
; (define run-n-jump (ptu 'right-stars (list (Word "run") (Word "jump"))))
; (ptu 'pair-count (car run-n-jump))

(define (add-dynamic-stars LLOBJ)
"
  add-dynamic-stars LLOBJ - Extend LLOBJ with row and column access
  methods (aka wildcard methods), specifically, to get all non-zero
  elements in a given row or column.

  Similar to the (add-pair-stars LLOBJ) class, except that this
  attempts to work without having to load all pairs into RAM at the
  same time; instead, pairs are fetched from the database dynamically,
  on demand.  This is attempting to address the problem where not all
  of the matrix can fit into RAM at the same time.  If all of the
  matrix can fit, then (add-pair-stars LLOBJ) is more efficient.

  The supported methods are:
  'left-basis - Return all items (atoms) that might be used to index
      a row in the matrix.  This may return more items than there are
      rows; no check is performed for empty or invalid rows. However,
      all valid rows will appear in the set: the returned set is a
      superset.  All of the elements of this set will be atoms of type
      (LLOBJ 'left-type).

  'right-basis - Likewise, but for columns.

  'left-stars COL - Returns pairs (*, COL), same as documented in the
  pair-stars API.

  'right-stars ROW - Likewise, but returns the set (ROW, *).
"
	(let ((stars-obj (add-pair-stars LLOBJ))
			(l-basis '())
			(r-basis '())
			(cache-incoming '())
			(pair-type (LLOBJ 'pair-type))
		)

		; Retreive all atoms of TYPE from the database
		(define (get-atoms TYPE)
			(load-atoms-of-type TYPE)
			(cog-get-atoms  TYPE))

		; Return a list of all items that might be rows.
		(define (get-left-basis)
			(if (null? l-basis)
				(set! l-basis (get-atoms (LLOBJ 'left-type))))
			l-basis)

		; Return a list of all items that might be columns.
		(define (get-right-basis)
			(if (null? r-basis)
				(set! r-basis (get-atoms (LLOBJ 'right-type))))
			r-basis)

		; Fetch the incoming set for ITEM, but only if we haven't
		; already done so.
		(define (get-incoming ITEM)
			(if (not (member ITEM cache-incoming))
				(begin
					(fetch-incoming-by-type ITEM pair-type)
					(set! cache-incoming (cons ITEM cache-incoming)))))

		; Return a list matrix entries with ITEM on the right;
		; that is, a wild-card on the left. That is, the set of
		; entries { (*, ITEM) }.  Uses the stars-obj to filter
		; the valid pairs, after fetching them from the database.
		(define (get-left-stars ITEM)
			(get-incoming ITEM)
			(stars-obj 'left-stars ITEM))

		(define (get-right-stars ITEM)
			(get-incoming ITEM)
			(stars-obj 'right-stars ITEM))

		;-------------------------------------------
		; Explain what it is that I provide. This helps classes
		; above understand what it is that this class does.
		(define (provides meth)
			(case meth
				((left-basis)       get-left-basis)
				((right-basis)      get-right-basis)
				((left-stars)       get-left-stars)
				((right-stars)      get-right-stars)
				(else               (LLOBJ 'provides meth))))

		;-------------------------------------------
		; Methods on this class.
		(lambda (message . args)
			(case message
				((left-basis)     (get-left-basis))
				((right-basis)    (get-right-basis))
				((left-stars)     (apply get-left-stars args))
				((right-stars)    (apply get-right-stars args))
				((provides)       (apply provides args))
				(else             (apply LLOBJ (cons message args))))
		)))

(define (do-it)
	(let ((pca (make-pseudo-cset-api))
			(psa (add-pair-stars pca))
			(pfa (add-pair-freq-api psa))
		)

		(load-atoms-of-type 'WordNode)

		; (define pta (make-thresh-pca pfa))
		; this can't work unless the whole matrix is already loaded...
		; (define all-words (get-all-cset-words))
		; brute force.
		; (define all-words (cog-get-atoms 'WordNode))

	)
)
