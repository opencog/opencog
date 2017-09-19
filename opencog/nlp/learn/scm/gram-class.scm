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
; Basic assumptions
; -----------------
; It is assummed that grammatical classes are stepping stones to word
; meaning; that meaning and grammatical class are at least partly
; correlated. It is assumed that words can have multiple meanings,
; and can belong to multiple grammatical classes. It is assumed that
; the sum total numner of observations of a word is a linear combination
; of the different ways that the word was used in the text sample.
; Thus, the task is to decompose the observed counts on a single word,
; and assign them to one of a number of different grammatical classes.
;
; The above implies that each word should be viewed as a vector; the
; disjuncts form the basis of the vector space, and the count of
; observations of different disjuncts indicating the direction of the
; vector. It is the linearity of the observations that implies that
; such a vector-based linear approach is correct.
;
; The number of grammatical classes that a word might belong to can
; vary from a few to a few dozen; in addition, there will be some
; unknown amount of "noise": incorrect sections due to incorrect parses.
;
; It is assumed that when a word belong to several grammatical classes,
; the sets of disjuncts defining those classes are not necessarily
; disjoint; there may be significant overlap.
;
; Merging
; -------
; There are several ways in which two words might be merged into a
; word-class, or a word added to a word-class. All of these are
; quasi-linear, trying to project out the different classes that
; the word belongs to.
;
; Union word-pair merging
; ------------------------
; Given two words, add them as vectors, creating a new vector, the
; word-class. This is purely linear summation. Next, compute the
; orthogonal components of the words to the word-class, and replace
; the words by thier orthogonal components - i.e. subtract the parallel
; components. It seems best to avoid negative observation counts, so
; if any count on any section is negative, it is clamped to zero (i.e.
; that section is removed, as this is a sparse vector). This last step
; renders this process only quasi-linear.
;
; Note the following properties of the above algo:
; a) The combined vector has strictly equal or larger support than
;    the parts. This might not be correct, as it seems that it will
;    mix in disjuncts that should have been assigned to other meanings.
; b) The process is not quite linear, as orthogonal components with
;    negative counts are clamped to zero.
; c) The number of vectors being tracked in the system is increasing:
;    before there were two, once for each word, now there are three:
;    each word remains, with altered counts, as well as thier sum.
;    It might be nice to prune the number of vectors, so that the
;    dataset does not get outrageously large. Its possible that short
;    vectors might be mostly noise.
; d) There is another non-linearity, when a word is assigned to an
;    existing word-class. This assignment will slightly alter the
;    direction of the word-class vector, but will not trigger the
;    recomputation of previous orthognoal components.
; e) The replacement of word-vectors by thier orthogonal components
;    means that the original word vectors are "lost". This could be
;    avoided by creating new "left-over" word vectors to hold just
;    the orthogonal components. However, this increases the size of
;    the dataset, and does not seem to serve any useful purpose.
;
; Overlap merging
; ---------------
; Similar to the above, a linear sum is taken, but the sum is only over
; those disjuncts that both words share in common. This might be more
; appropriate for disentangling linear combinaations of multiple
; word-senes. It seems like it could be robust even with lower
; similarity scores (e.g. when using cosine similarity).
;
; Overlap merging appears to solve the problem a) above, but. on the
; flip side, it also seems to prevent the discovery and broadening
; of the ways in which a word might be used.
;
; merge-ortho
; -----------
; The above two merge methods are implemented in the `merge-ortho`
; function. It takes, as an argument, a fractional weight which is
; used when the disjunct isn't shared between both words. Setting
; the weight to zero gives overlap merging; setting it ton one gives
; union merging.
;
; Broadening
; ----------
; The issue described in a) is an issue of broadening the known usages
; of a word, beyond what has been strictly observed in the text.  There
; are two distinct opportunities to broaden: first, in the union vs.
; overlap merging above, and second, in the merging of disjuncts. That
; is, the above merging did not alter the number of disjuncts in use:
; its disjuncts is a sequence of connectors, each connector specifies
; a single word. At some point, disjuncts should also be merged, i.e.
; by merging the connectors on them.
;
; If disjunct merging is performed after a series of word mergers have
; been done, then when a connector-word is replaced by a connector
; word-class, that class may be larger than the number of connectors
; originally witnessed. Again, the known usage of the word is broaded.
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
; Property g) above seems wrong: word-classes should appeary fully
; connected in the graph, symmetrically.  This suggests a disjunct
; merger style that maintains connectivity.
;
; As above, given a single word, one scans the sections on, looking
; for sections that differ in only one location. As before, the words
; that appear at this variable location are tossed into a set. However,
; this time, a search is made to see if this set overlaps, or is
; a subset of an existing grammatical class. If so, then the counts
; on all of these sections are totalled, a new disjunct is created,
; using the grammatical class in the connector, and the individual
; sections are discarded. (If there are multiple grammatical classes
; that might be appropriate, then a cosine similarity could be used
; to pick between them.
;
; This has the nice property:
; h) The total number of sections is decreasing.
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix) (opencog persist))

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

  This overloads the standard add-pair-stars methods, so that the
  fetching occurs automatically and transparently to the user.

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

  'left-release COL - Remove pairs (*, COL) from the atomspace. This
  intended to be used to minimize RAM usage when working with a large
  database.  The atoms are NOT removed from the database; only from
  the atospace. If the atoms are in use (have a non-empty incoming
  set) they are not removed.

  'right-release ROW - same but for pairs in ROW.
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
		; Release (extract) row or column. No spcific check is made
		; to really be sure that this is a part of the matrix; it's
		; assumed that the pair-type is enough to acheive this.
		(define (release-extract ITEM)
			(for-each cog-extract (cog-incoming-by-type ITEM pair-type)))

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
				((left-release)   (apply release-extract args))
				((right-release)  (apply release-extract args))
				((provides)       (apply provides args))
				(else             (apply LLOBJ (cons message args))))
		)))

; ---------------------------------------------------------------------

(define (merge-ortho LLOBJ WA WB FRAC)
"
  merge-ortho WA WB FRAC - merge WA and WB into a grammatical class.
  Return the merged class.

  WA and WB should be WordNodes or WordClassNodes.
  FRAC should be a floating point nummber between zero and one,
     indicating the fraction of a non-shared count to be used.
     Setting this to 1.0 gives the sum of the union of supports;
     setting this to 0.0 gives the sum of the intersection of supports.
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

  The counts are summed only if both counts are non-zero. Otherwise,
  only a WEIGHT fraction of a single, unmatched count is transfered.

  If WA is a WordClassNode, and WB is not, then WB is merged into
  WA. Currently, WB must never be a WordClass....
"
	; set-count ATOM CNT - Set the raw observational count on ATOM.
	(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 1 0 CNT)))

	; Create a new word-class out of the two words.
	; Concatenate the string names to get the class name.
	; If WA is already a word-class, just use it as-is.
	(define wrd-class
		(if (eq? 'WordClassNode (cog-type WA)) WA
			(WordClassNode (string-concatenate
					(list (cog-name WA) " " (cog-name WB))))))

	; Merge two sections into one section built from the word-class.
	; One or the other sections can be null. If both sections are not
	; null, then both are assumed to have exactly the same disjunct.
	;
	; This works fine for merging two words, or for merging
	; a word and a word-class.  It even works for merging
	; two word-classes.
	;
	; This is a fold-helper; the fold accumulates the length-squared
	; of the merged vector.
	(define (merge-word-pair SECT-PAIR LENSQ)
		; The two word-sections to merge
		(define lsec (first SECT-PAIR))
		(define rsec (second SECT-PAIR))

		; The counts on each, or zero.
		(define lcnt (if (null? lsec) 0 (LLOBJ 'pair-count lsec)))
		(define rcnt (if (null? rsec) 0 (LLOBJ 'pair-count rsec)))

		; Return #t if sect is a Word section, not a word-class section.
		(define (is-word-sect? sect)
			(eq? 'WordNode (cog-type (cog-outgoing-atom sect 0))))

		; If the other count is zero, take only a FRAC of the count.
		; But only if we are merging in a word, not a word-class;
		; we never want to shrink the support of a word-class, here.
		(define wlc (if
				(and (null? rsec) (is-word-sect? lsec))
				(* FRAC lcnt) lcnt))
		(define wrc (if
				(and (null? lsec) (is-word-sect? rsec))
				(* FRAC rcnt) rcnt))

		; Sum them.
		(define cnt (+ wlc wrc))

		; The disjunct. Both lsec and rsec have the same disjunct.
		(define seq (if (null? lsec) (cog-outgoing-atom rsec 1)
				(cog-outgoing-atom lsec 1)))

		; The merged word-class
		(define mrg (Section wrd-class seq))

		; The summed counts
		(set-count mrg cnt)
		(store-atom mrg) ; save to the database.

		; Return the accumulated sum-square length
		(+ LENSQ (* cnt cnt))
	)

	; The length-squared of the merged vector.
	(define lensq
		(fold merge-word-pair 0.0 (ptu 'right-stars (list WA WB))))

	; Given a WordClassNode CLS and a WordNode WRD, alter the
	; counts on the disjuncts on WRD, so that they are orthogonal
	; to CLS.  If the counts are negative, that word-disjunct pair
	; is deleted (from the database as well as the atomspace).
	; The updated counts are stored in the database.
	;
	; NOTE: actually, both CLS and WRD are Sections, its just that
	; CLS is a section for a WordClassNode, and WRD is a section for
	; a WordNode.  The sectinos hold the counts, of course.
	(define (orthogonalize CLS WRD)

		; Fold-helper to compute the dot-product between the WRD
		; vector and the CLS vector.
		(define (compute-dot-prod CLAPR DOT-PROD)
			(define cla (first CLAPR))
			(define wrd (second CLAPR))

			; The counts on each, or zero.
			(define cc (LLOBJ 'pair-count cla))
			(define wc (if (null? wrd) 0 (LLOBJ 'pair-count wrd)))

			(+ DOT-PROD (* cc wc))
		)

		; Compute the dot-product of WA and the merged class.
		; We could use fold here, but we won't.
		(define dot-prod
			(fold compute-dot-prod 0.0 (ptu 'right-stars (list CLS WRD))))
		(define unit-prod (/ dot-prod lensq))

		; (format #t "sum ~A dot-prod=~A length=~A unit=~A\n"
		;      WRD dot-prod lensq unit-prod)

		; Alter the counts on the word so that they are orthogonal
		; to the class. Assumes that the dot-prduct was previously
		; computed, and also that the mean-square length of the
		; class was also previously computed.
		(define (ortho CLAPR)
			(define cla (first CLAPR))
			(define wrd (second CLAPR))

			; The counts on each, or zero.
			; Both cla and wrd are actually Sections.
			(define cc (LLOBJ 'pair-count cla))
			(define wc (if (null? wrd) 0 (LLOBJ 'pair-count wrd)))

			; The orthogonal component.
			(define orth (if (null? wrd) -999
					(- wc (* cc unit-prod))))

			; Update count on postive sections;
			; Delete non-postive sections. The deletion is not just
			; from the atomspace, but also the database backend!
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

			(orthogonalize wrd-class WA)
			(orthogonalize wrd-class WB))

		; If WA is not a WordNode, assume its a WordClassNode.
		; The process is similar, but slightly altered.
		; We assume that WB is a WordNode, but perform no safety
		; checking to verify this.
		(begin
			; Add WB to the mrg-class (which is WA already)
			(MemberLink WB wrd-class)

			; Redefine WB to be orthogonal to the word-class.
			(orthogonalize wrd-class WB))
	)
	wrd-class
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
		(if (eq? 1 cnt) mismatch-idx #f))

	; Return false if they are equal, else return the index
	(if (or (eq? SEQA SEQB) (not (eq? (cog-arity SEQA) (cog-arity SEQB))))
		 #f
		 (get-idx))
)


; ---------------------------------------------------------------
; stub wrapper for word-similarity.
; Return #t if the two should be merged, else return #f
; WORD-A might be a WordClassNode or a WordNode.
; XXX do something fancy here.
(define (ok-to-merge WORD-A WORD-B)
	; (define pca (make-pseudo-cset-api))
	; (define psa (add-dynamic-stars pca))
	; (define pcos (add-pair-cosine-compute psa))

	(define sim (pcos 'right-cosine WORD-A WORD-B))
(format #t "Cosine=~A for \"~A\" -- \"~A\"\n" sim (cog-name WORD-A) (cog-name WORD-B))

	; True, if sim is more than 0.9
	(< 0.9 sim)
)

; ---------------------------------------------------------------
;
; Given just one word, assemble a list of all of the words that
; appear in the disjuncts on that word.  Assumes that the disjuncts
; for the word are already in the atomspace.
(define (get-dj-words WORD)

	; Given a Section i.e. (word, connector-set), walk over all
	; the connectors in the connector set, and add the word appearing
	; in the connector to the word-list. But add it only if it is not
	; already in the list.
	(define (add-to-list SEC WORD-LIST)
		(fold
			(lambda (CNCTR LST)
				(define WRD (cog-outgoing-atom CNCTR 0))
				(if ; (and
					; Is it actually a word (and not a word-class?)
					(eq? 'WordNode (cog-type WRD))
					; Is it not yet in the list?
					; Its not efficient to check here; this becomes very
					; slow for long lists - it's O(N^2)
					; (not (find (lambda (wrd) (equal? WRD wrd)) LST)))
					(cons WRD LST) LST))
			WORD-LIST
			; second atom of Section is a ConnectorSeq
			(cog-outgoing-set (cog-outgoing-atom SEC 1))))

	; Walk over all the Sections on the word.
	(define all-words
		(fold add-to-list '() (cog-incoming-by-type WORD 'Section)))

	; It is faster, more cpu-efficient to sort first, and then filter.
	(define sorted-words (sort all-words cog-atom-less?))
	(if (null? sorted-words) '()
		(fold
			(lambda (WRD LST)
				(if (equal? WRD (car LST)) LST (cons WRD LST)))
			(car sorted-words)
			(cdr sorted-words)))
)

; ---------------------------------------------------------------
; Predicate - is the word a member of the grammatical class, already?
(define (in-gram-class? WORD GCLS)
	(define memlnk (cog-link 'MemberLink WORD GCLS))
	(if (null? memlnk) #f #t))

; ---------------------------------------------------------------
; Given a grammatical class, and a list of words, scan the word list
; to see if any can be merged into the class. If they can, merge them
; in. This is an O(n) algo.
;
; GRM-CLS should be the WordClassNode to be enlarged.
; WRD-LST should be list of WordNodes to compare to the class.
; LLOBJ is the object to use for obtaining counts.
; FRAC is the fraction of union vs. intersection during merge.
; (These last two are passed blindly to the merge function).
;
(define (expand-gram-class LLOBJ GRM-CLS WRD-LST FRAC)
	(if (not (null? WRD-LST))
		(let ((wrd (car WRD-LST)))

			; If the word is not already in the class, and its mergeable,
			; then merge it. XXX Do we really need the in-gram-class?
			; check? Probably not ... if its in the class, then what's
			; left is not mergable.
			(if (and (not (in-gram-class? wrd GRM-CLS))
					(ok-to-merge GRM-CLS wrd))
				(merge-ortho LLOBJ GRM-CLS wrd FRAC))
			(expand-gram-class LLOBJ GRM-CLS (cdr WRD-LST) FRAC)))
)

; ---------------------------------------------------------------
; Given a list of words, compare them pairwise to find a similar
; pair. Merge these to form a grammatical class, and then try to
; expand that class as much as possible.
(define (make-gram-class LLOBJ WRD-LST FRAC)

	; Try to make a grammatical class out of the word, and another
	; in the list. Return it, if possible, else return nil.
	(define (make-first-one word rest)
		(if (null? rest) '()
			(if (ok-to-merge word (car rest))
				(merge-ortho LLOBJ word (car rest) FRAC)
				; If they are NOT mergable, recurse,
				; looking for a pair that is.
				(make-first-one (car rest) (cdr rest)))))

	(define grm-class (make-first-one (car WRD-LST) (cdr WRD-LST)))
	(if (not (null? grm-class))
		(expand-gram-class LLOBJ grm-class WRD-LST FRAC))
)

; ---------------------------------------------------------------

(define (do-it)
	(let ((pca (make-pseudo-cset-api))
			(psa (add-dynamic-stars pca))
			(pcos (add-pair-cosine-compute psa))
		)

		(load-atoms-of-type 'WordNode)
		; Verify that words have been loaded
		;  (define all-words (get-all-cset-words))
	)
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
