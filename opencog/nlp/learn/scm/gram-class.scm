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

; ---------------------------------------------------------------------
; TODO: move the below to the atomsace matrix directory, when its
; ready. Its not ready, it needs to impleemnt extract, to unclog RAM.
;

(define-public (add-dynamic-stars LLOBJ)
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

; ---------------------------------------------------------------------

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

  If WA is a WordClassNode, and WB is not, then WB is merged into
  WA. Currently, WB must never be a WordClass....
"
	; set-count ATOM CNT - Set the raw observational count on ATOM.
	(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 1 0 CNT)))

	; Create a new word-class out of the two words.
	(define wrd-class
		(if (eq? 'WordClassNode (cog-type WA)) WA
			(WordClassNode (string-concatenate
					(list (cog-name WA) " " (cog-name WB))))))

	; The length-squared of the merge vector.
	(define lensq 0.0)

	; Merge two words into a word-class.
	; This works fine for merging two words, or for merging
	; a word and a word-class.  It even worsk for merging
	; two word-classes.
	(define (merge-word-pair WORD-PAIR)
		; The two words to merge
		(define lw (first WORD-PAIR))
		(define rw (second WORD-PAIR))

		; The counts on each, or zero.
		(define lc (if (null? lw) 0 (LLOBJ 'pair-count lw)))
		(define rc (if (null? rw) 0 (LLOBJ 'pair-count rw)))
		(define cnt (+ rc lc))

		; The disjunct. Both lw and rw have the same disjunct.
		(define seq (if (null? lw) (cog-outgoing-atom rw 1)
				(cog-outgoing-atom lw 1)))

		; The merged word-class
		(define mrg (Section wrd-class seq))

		; Update the sum-square length
		(set! lensq (+ lensq (* cnt cnt)))

		; The summed counts
		(set-count mrg cnt)

		(store-atom mrg) ; save to SQL
	)

	; Given a WordClassNode CLS and a WordNode WRD, alter the
	; counts on the disjuncts on WRD, so that they are orthogonal
	; to CLS.  If the counts are negative, that word-disjunct pair
	; is deleted (from the database as well as the atomspace).
	; The updated counts are stored in the database.
	(define (orthogonalize CLS WRD)

		; Machinery to compute the dot-product between the WRD
		; vector and the CLS vector.
		(define dot-prod 0.0)
		(define (compute-dot-prod CLAPR)
			(define cla (first CLAPR))
			(define wrd (second CLAPR))

			; The counts on each, or zero.
			(define cc (LLOBJ 'pair-count cla))
			(define wc (if (null? wrd) 0 (LLOBJ 'pair-count wrd)))

			(set! dot-prod (+ dot-prod (* cc wc)))
		)

		; Alter the counts on the word so that they are orthogonal
		; to the class. Assumes that the dot-prduct was previously
		; computed, and also that the mean-square length of the
		; class was also previously computed.
		(define (ortho CLAPR)
			(define cla (first CLAPR))
			(define wrd (second CLAPR))

			; The counts on each, or zero.
			(define cc (LLOBJ 'pair-count cla))
			(define wc (if (null? wrd) 0 (LLOBJ 'pair-count wrd)))

			; The orthogonal component.
			(define orth (if (null? wrd) -999
					(- wc (* cc dot-prod))))

			; Update count on postive sections;
			; Delete non-postive sections.
			(if (< 0 orth)
				(set-count wrd orth)
				(if (not (null? wrd))
					(begin
						; Set to 0 just in case the delete below can't happen.
						(set-count wrd 0)
						(cog-delete wrd))))

			; Update the database.
			(if (cog-atom? wrd) (store-atom wrd))

			; (if (< 3 orth) (format #t "Large remainder: ~A\n" wrd))
		)

		; Compute the dot-product of WA and the merged class.
		; We could use fold here, but we won't.
		(for-each compute-dot-prod (ptu 'right-stars (list CLS WRD)))

; (format #t "sum ~A dotty ~A lens ~A\n" WRD dot-prod lensq)
		(set! dot-prod (/ dot-prod lensq))
; (format #t "final dotty ~A\n" dot-prod)

		; Compute the orthogonal components
		(for-each ortho (ptu 'right-stars (list CLS WRD)))
	)

	(define psa (add-dynamic-stars LLOBJ))
	(define (bogus a b) (format #t "Its ~A and ~A\n" a b))
	(define ptu (add-tuple-math LLOBJ bogus))


	(if (eq? 'WordNode (cog-type WA))
		(begin

			; Put the two words into the new word-class.
			(MemberLink WA wrd-class)
			(MemberLink WB wrd-class)

			; Create the merged vector.
			(for-each merge-word-pair (ptu 'right-stars (list WA WB)))

			(orthogonalize wrd-class WA)
			(orthogonalize wrd-class WB))

		; If WA is not a WordNode, assume its a WordClassNode.
		; The process is similar, but slightly altered.
		; We assume that WB is a WordNode, but perform no safety
		; checking to verify this.
		(begin
			; Add WB to the mrg-class (which is WA already)
			(MemberLink WB wrd-class)

			; Merge WB into the word-class
			(for-each merge-word-pair (ptu 'right-stars (list WA WB)))

			; Redefine WB to be orthogonal to the word-class.
			(orthogonalize wrd-class WB))
	)
)

; ---------------------------------------------------------------
(define (do-it)
	(let ((pca (make-pseudo-cset-api))
			(psa (add-dynamic-stars pca))
		)

		(load-atoms-of-type 'WordNode)

		; (define pta (make-thresh-pca pfa))
		; this can't work unless the whole matrix is already loaded...
		; (define all-words (get-all-cset-words))
		; brute force.
		; (define all-words (cog-get-atoms 'WordNode))

	)
)

; ---------------------------------------------------------------
; Example usage
;
; (define psa (add-dynamic-stars pca))
;
; (define (bogus a b) (format #t "Its ~A and ~A\n" a b))
; (define ptu (add-tuple-math psa bogus))
; (define run-n-jump (ptu 'right-stars (list (Word "run") (Word "jump"))))
; (ptu 'pair-count (car run-n-jump))
;
; (merge-ortho psa (Word "run") (Word "jump"))
