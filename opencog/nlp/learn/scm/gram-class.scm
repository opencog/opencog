;
; gram-class.scm
;
; Compare words and word-classes by grammatical similarities.
;
; Copyright (c) 2017, 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; When a pair of words are judged to be grammatically similar, they
; can be used to create a "grammatical class", containing both the
; words, and behaving as their average.  Similarly, a word can be
; compared to an existing grammatical class, to see if it belongs to
; that class.  This file implements several different systems for
; comparing the grammatical similarity of words and word-classes.
;
; Representation
; --------------
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
; The TV on the MemberLink holds a count value; that count equals the
; total number of section-counts that were transfered from the word, to
; the word-class, when the word was merged into the class. The sum over
; all of these counts (on the MemberLinks) should exactly equal the sum
; over the counts on all Sections for that WordClassNode.  Thus, it can
; be used to determine what fraction the word contributed to the class.
;
; Basic assumptions
; -----------------
; It is assumed that grammatical classes are stepping stones to word
; meaning; that meaning and grammatical class are at least partly
; correlated. It is assumed that words can have multiple meanings, and
; thus can belong to multiple grammatical classes. It is assumed that
; the sum total number of observations of a word is a linear combination
; of the different ways that the word was used in the text sample.
; Thus, the task is to decompose the observed counts on a single word,
; and assign them to one of several different grammatical classes.
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
; It is assumed that when a word belongs to several grammatical classes,
; the sets of disjuncts defining those classes are not necessarily
; disjoint; there may be significant overlap. That is, different
; grammatical classes are not orthogonal, in general.
;
;
; Semantic disambiguation
; -----------------------
; The correct notion of a grammatical class is not so much as a
; collection of words, but rather as a collection of word-senses.
; Consider the word "saw": it can be the past tense of the verb
; "to see", or it can be the cutting tool, a noun.  Thus, the word
; "saw" should belong to at least two different grammatical classes.
; The actual word-sense is "hidden", only the actual word is observed.
; The "hidden" word-sense can be partly (or mostly) discerned by looking
; at how the word was used: nouns are used differently than verbs.
; The different usage is reflected in the collection of sections
; ("disjuncts") that are associated with the word-sense.
;
; Thus, the vector associated to the word "saw" is the (linear) sum
; for a noun-vector (the cutting tool) and two different verb-vector
; (observing; cutting).  This section describes how the cosine-distance
; can be used to distinguish between these different forms, how to
; factor the vector of observation counts into distinct classes.
;
;
; Word Similarity
; ---------------
; There are several different means of comparing similarity between
; two words.  The simplest is cosine distance: if the cosine of two
; word-vectors is greater than a threshold, they should be merged.
;
; The cosine distance between the two words w_a, w_b is
;
;    cos(w_a, w_b) = v_a . v_b / |v_a||v_b|
;
; Where, as usual, v_a . v_b is the dot product, and |v| is the length.
;
; The minimum-allowed cosine-distance is a user-tunable parameter in
; the code below; it is currently hard-coded to 0.65.
;
; Other similarity measures are possible, but have not yet been
; explored.
;
;
; Merge Algos
; -----------
; There are several ways in which two words might be merged into a
; word-class, or a word added to a word-class.  Some of these are
; described below.  Additional kind of merges can be imagined; how to
; accurately judge the quality of different approaches is unclear.
;
;
; Union word-pair merging
; ------------------------
; Given two words, add them as vectors, creating a new vector, the
; word-class. This is purely linear summation. Next, compute the
; orthogonal components of the words to the word-class, and replace
; the words by their orthogonal components - i.e. subtract the parallel
; components. It seems best to avoid negative observation counts, so
; if any count on any section is negative, it is clamped to zero (i.e.
; that section is removed, as this is a sparse vector). This last step
; renders this process only quasi-linear.
;
; Note the following properties of this algo:
; a) The combined vector has strictly equal or larger support than
;    the parts. This might not be correct, as it seems that it will
;    mix in disjuncts that should have been assigned to other meanings.
;    (the SUPPORT issue; discussed further below).
; b) The process is not quite linear, as orthogonal components with
;    negative counts are clamped to zero.
;    (the LEXICAL issue; discussed further, below)
; c) The number of vectors being tracked in the system is increasing:
;    before there were two, once for each word, now there are three:
;    each word remains, with altered counts, as well as their sum.
;    It might be nice to prune the number of vectors, so that the
;    dataset does not get outrageously large. Its possible that short
;    vectors might be mostly noise.
; d) There is another non-linearity, when a word is assigned to an
;    existing word-class. This assignment will slightly alter the
;    direction of the word-class vector, but will not trigger the
;    recomputation of previous orthogonal components.
; e) The replacement of word-vectors by their orthogonal components
;    means that the original word vectors are "lost". This could be
;    avoided by creating new "left-over" word vectors to hold just
;    the orthogonal components. However, this increases the size of
;    the dataset, and does not seem to serve any useful purpose.
;
;
; Overlap merging
; ---------------
; Similar to the above, a linear sum is taken, but the sum is only over
; those disjuncts that both words share in common. This might be more
; appropriate for disentangling linear combinations of multiple
; word-senses. It seems like it could be robust even with lower
; similarity scores (e.g. when using cosine similarity).
;
; Overlap merging appears to solve the problem a) above (the SUPPORT
; issue), but, on the flip side, it also seems to prevent the discovery
; and broadening of the ways in which a word might be used. (Broadening
; is duscuseed in greater detail below.)
;
; Formal definition
; -----------------
; A formal (equational, algorithmic) description of overlap merging is
; given here. One wishes to compute the intersection of basis elements
; (the intersection of "disjuncts" aka "sections") of the two words, and
; then sum the counts only on this intersected set. Let
;
;   {e_a} = set of basis elements in v_a with non-zero coefficients
;   {e_b} = set of basis elements in v_b with non-zero coefficients
;   {e_overlap} = {e_a} set-intersection {e_b}
;   pi_overlap = unit on diagonal for each e in {e_overlap}
;              == projection matrix onto the subspace {e_overlap}
;   v_a^pi = pi_overlap . v_a == projection of v_a onto {e_overlap}
;   v_b^pi = pi_overlap . v_b == projection of v_b onto {e_overlap}
;
;   v_cluster = v_a^pi + v_b^pi
;   v_a^new = v_a - v_a^pi
;   v_b^new = v_b - v_b^pi
;
; The idea here is that the vector subspace {e_overlap} consists of
; those grammatical usages that are common for both words a and b,
; and thus hopefully correspond to how words a and b are used in a
; common sense. Thus v_cluster is the common word-sense, while v_a^new
; and v_b^new are everything else, everything left-over.  Note that
; v_a^new and v_b^new are orthogonal to v_cluster. Note that v_a^new
; and v_b^new are both exactly zero on {e_overlap} -- the subtraction
; wipes out those coefficents. Note that the total number of counts
; is preserved.  That is,
;
;   ||v_a|| + ||v_b|| = ||v_cluster|| + ||v_a^new|| + ||v_b^new||
;
; where ||v|| == ||v||_1 the l_1 norm aka count aka manhattan-distance.
;
; If v_a and v_b have several word-senses in common; then so will
; v_cluster.  Since there is no a priori way to force v_a and v_b to
; encode only one common word sense, there needs to be some distinct
; machanism to split v_cluster ino multiple word senses, if that is
; needed.
;
; Union merging can be descrbied using almost the same formulas, except
; that one takes
;
;   {e_union} = {e_a} set-union {e_b}
;
; merge-project
; -------------
; The above two merge methods are implemented in the `merge-project`
; function. It takes, as an argument, a fractional weight which is
; used when the disjunct isn't shared between both words. Setting
; the weight to zero gives overlap merging; setting it to one gives
; union merging. Setting it to fractional values provides a merge
; that is intermediate between the two: an overlap, plus a bit more,
; viz some of the union.
;
; That is, the merger is given by the vector
;
;   v_merged = v_overlap + FRAC * (v_union - v_overlap)
;
; If v_a and v_b are both words, then the counts on v_a and v_b are
; adjusted to remove the counts that were added into v_merged. If one
; of the two is already a word-class, then the counts are simply moved
; from the word to the class.
;
; merge-ortho
; -----------
; The `merge-ortho` function computes the merged vector the same way as
; the `merge-project` function; however, it adjusts counts on v_a and
; v_b in a different way. What it does is to explicitly orthogonalize
; so that the final v_a and v_b are orthogonal to v_merged.  The result
; is probably very similar to `merge-project`, but not the same.  In
; particular, the total number of counts in the system is not preserved
; (which is maybe a bad thing?)
;
; There's no good reason for choosing `merge-ortho` over `merge-project`,
; and there's a good theoretical reason to not use `merge-ortho`. Its
; here for historical reasons -- it got coded funky, and that's that.
; It should probably be eventually removed.
;
;
; merge-discrim
; -------------
; Built on the merge-prject method, the FRAC is a sigmoid function,
; ranging from 0.0 to 1.0, depending on the cosine between the vectors.
; The idea is that, if two words are already extremely similar, we may
; as well assum they really are in the same class, and so do a union
; merge. But if the words are only kind-of similar, but not a lot, then
; assume that the are both linear combinations of several word senses,
; and do only the minimal overlap merge, so as to avoid damaging the
; other senses.
;
; A reasonable strategy would seem to bee to take
;
;   FRAC = (cos - cos_min) / (1.0 = cos_min)
;
; where cos_min is the minimum cosine acceptable, for any kind of
; merging to be performed.
;
;
; Parameter choices
; -----------------
; Gut-sense intuition suggests these possible experiments:
;
; * Fuzz: use `merge-project` with hard-coded frac=0.3 and min acceptable
;   cosine=0.65
;
; * Discrim: use `merge-discrim` with min acceptable cosine = 0.5
;
;
; Broadening
; ----------
; The issue described in a) is an issue of broadening the known usages
; of a word, beyond what has been strictly observed in the text.  There
; are two distinct opportunities to broaden: first, in the union vs.
; overlap merging above, and second, in the merging of disjuncts. That
; is, the above merging did not alter the number of disjuncts in use:
; the disjuncts on the merged class are still disjuncts with single-word
; connectors. At some point, disjuncts should also be merged, i.e. by
; merging the connectors on them.
;
; If disjunct merging is performed after a series of word mergers have
; been done, then when a connector-word is replaced by a connector
; word-class, that class may be larger than the number of connectors
; originally witnessed. Again, the known usage of the word is broadened.
;
;
; Other Merge Strategies
; ----------------------
; Insofar as the the "hidden" meanings of words control they way they
; are used in sentences, it is plausible to assume that perhaps a Hidden
; Markov Model (HMM) style approach might provide an alternative way of
; splitting vectors into distinct parts.  Alternately, an Artificial
; Neural Nets (ANN), possibly with deep-learning, might provide a better
; factorization. At this time, these remain unexplored.
;
;
; Disjunct merging
; ----------------
; Disjunct merging is the second step in creating grammatical classes.
; The idea here is to replace individual connectors that specify words
; with connectors that specify word-classes. This step is examined in
; greater detail in `cset-class.scm`.
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix) (opencog persist))

; ---------------------------------------------------------------------

(define (merge-project LLOBJ FRAC WA WB)
"
  merge-project FRAC WA WB - merge WA and WB into a grammatical class.
  Return the merged class.

  WA should be a WordNode or a WordClassNode.
  WB is expected to be a WordNode.
  FRAC should be a floating point number between zero and one,
     indicating the fraction of a non-shared count to be used.
     Setting this to 1.0 gives the sum of the union of supports;
     setting this to 0.0 gives the sum of the intersection of supports.
  LLOBJ is used to access counts on pairs.  Pairs are SectionLinks,
     that is, are (word,disjunct) pairs wrapped in a SectionLink.

  The merger of WA and WB are performed, using the 'projection
  merge' strategy. This is done like so. If WA and WB are both
  WordNodes, then a WordClass is created, having both WA and WB as
  members.  Counts are then transfered from WA and WB to the class.

  The counts are summed only if both counts are non-zero. Otherwise,
  only a FRAC fraction of a single, unmatched count is transferred.

  If WA is a WordClassNode, and WB is not, then WB is merged into
  WA. That is, the counts on QA are adjusted only upwards, and those
  on WB only downwards.
"
	(define (bogus a b) (format #t "Its ~A and ~A\n" a b))
	(define ptu (add-tuple-math LLOBJ bogus))

	; set-count ATOM CNT - Set the raw observational count on ATOM.
	(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 1 0 CNT)))

	; Create a new word-class out of the two words.
	; Concatenate the string names to get the class name.
	; If WA is already a word-class, just use it as-is.
	(define wrd-class
		(if (eq? 'WordClassNode (cog-type WA)) WA
			(WordClassNode (string-concatenate
					(list (cog-name WA) " " (cog-name WB))))))

	; Accumulated counts for the two.
	(define accum-lcnt 0)
	(define accum-rcnt 0)

	; Merge two sections into one, placing the result on the word-class.
	; Given, given a pair of sections, sum the counts from each, and
	; then place that count on a corresponding section on the word-class.
	; Store the updated section to the database.
	;
	; One or the other sections can be null. If both sections are not
	; null, then both are assumed to have exactly the same disjunct.
	;
	; This works fine for merging two words, or for merging
	; a word and a word-class.  It even works for merging
	; two word-classes.
	;
	(define (merge-section-pair SECT-PAIR)
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

		; Compute what's left on each.
		(define lrem (- lcnt wlc))

		; Update the count on the section.
		; If the count is zero or less, delete the section.
		(define (update-section-count SECT CNT)
			(if (< 1.0e-10 CNT)
				(begin (set-count SECT CNT) (store-atom SECT))
				(begin (set-count SECT 0) (cog-delete SECT))))

		; The cnt can be zero, if FRAC is zero.  Do nothing in this case.
		(if (< 1.0e-10 cnt)
			(let* (
					; The disjunct. Both lsec and rsec have the same disjunct.
					(seq (if (null? lsec) (cog-outgoing-atom rsec 1)
							(cog-outgoing-atom lsec 1)))
					; The merged word-class
					(mrg (Section wrd-class seq))
				)

				; The summed counts
				(set-count mrg cnt)
				(store-atom mrg) ; save to the database.

				; Now subtract the counts from the words.
				; Left side is either a word or a word-class.
				; If its a word-class, we've already updated
				; the count.
				(if (and (not (null? lsec)) (is-word-sect? lsec))
					(update-section-count lsec (- lcnt wlc)))

				; Right side is WB and is always a WordNode
				(if (not (null? rsec))
					(update-section-count rsec (- rcnt wrc)))
			))

		; Accumulate the counts, handy for tracking membership fraction
		(set! accum-lcnt (+ accum-lcnt wlc))
		(set! accum-rcnt (+ accum-rcnt wrc))
	)

	(for-each merge-section-pair (ptu 'right-stars (list WA WB)))

	(if (eq? 'WordNode (cog-type WA))
		(let ((ma (MemberLink WA wrd-class))
				(mb (MemberLink WB wrd-class)))
			; Track the number of word-observations moved from
			; the words, the the class. This is how much the words
			; contributed to the class.
			(set-count ma accum-lcnt)
			(set-count mb accum-rcnt)
			; Put the two words into the new word-class.
			(store-atom ma)
			(store-atom mb))

		; If WA is not a WordNode, assume its a WordClassNode.
		; The process is similar, but slightly altered.
		; We assume that WB is a WordNode, but perform no safety
		; checking to verify this.
		(let ((mb (MemberLink WB wrd-class)))
			(set-count mb accum-rcnt)
			; Add WB to the mrg-class (which is WA already)
			(store-atom mb))
	)
	wrd-class
)

; ---------------------------------------------------------------------

(define (merge-ortho LLOBJ FRAC WA WB)
"
  DEPRECATED! Use merge-project instead!

  merge-ortho FRAC WA WB - merge WA and WB into a grammatical class.
  Return the merged class.

  WA should be a WordNode or a WordClassNode.
  WB is expected to be a WordNode.
  FRAC should be a floating point number between zero and one,
     indicating the fraction of a non-shared count to be used.
     Setting this to 1.0 gives the sum of the union of supports;
     setting this to 0.0 gives the sum of the intersection of supports.
  LLOBJ is used to access counts on pairs.  Pairs are SectionLinks,
     that is, are (word,disjunct) pairs wrapped in a SectionLink.

  The merger of WA and WB are performed, using the 'orthogonal
  merge' strategy. This is done like so. If WA and WB are both
  WordNodes, then a WordClass is created, having both WA and WB as
  members.  The counts on that word-class are the sum of the counts
  on WA and WB. Next, the counts on WA and WB are adjusted, so that
  only the orthogonal components are left (that is, the parts
  orthogonal to the sum). Next, zero-clamping is applied, so that
  any non-positive components are erased.

  The counts are summed only if both counts are non-zero. Otherwise,
  only a FRAC fraction of a single, unmatched count is transferred.

  If WA is a WordClassNode, and WB is not, then WB is merged into
  WA.
"
	(define (bogus a b) (format #t "Its ~A and ~A\n" a b))
	(define ptu (add-tuple-math LLOBJ bogus))

	; set-count ATOM CNT - Set the raw observational count on ATOM.
	(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 1 0 CNT)))

	; Create a new word-class out of the two words.
	; Concatenate the string names to get the class name.
	; If WA is already a word-class, just use it as-is.
	(define wrd-class
		(if (eq? 'WordClassNode (cog-type WA)) WA
			(WordClassNode (string-concatenate
					(list (cog-name WA) " " (cog-name WB))))))

	; Merge two sections into one, placing the result on the word-class.
	; Given, given a pair of sections, sum the counts from each, and
	; then place that count on a corresponding section on the word-class.
	; Store the updated section to the database.
	;
	; One or the other sections can be null. If both sections are not
	; null, then both are assumed to have exactly the same disjunct.
	;
	; This works fine for merging two words, or for merging
	; a word and a word-class.  It even works for merging
	; two word-classes.
	;
	; This is a fold-helper; the fold accumulates the length-squared
	; of the merged vector.
	(define (merge-section-pair SECT-PAIR LENSQ)
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

		; The cnt can be zero, if FRAC is zero.  Do nothing in this case.
		(if (< 0 cnt)
			(let* (
					; The disjunct. Both lsec and rsec have the same disjunct.
					(seq (if (null? lsec) (cog-outgoing-atom rsec 1)
							(cog-outgoing-atom lsec 1)))
					; The merged word-class
					(mrg (Section wrd-class seq))
				)

				; The summed counts
				(set-count mrg cnt)
				(store-atom mrg) ; save to the database.
			))

		; Return the accumulated sum-square length
		(+ LENSQ (* cnt cnt))
	)

	; The length-squared of the merged vector.
	(define lensq
		(fold merge-section-pair 0.0 (ptu 'right-stars (list WA WB))))

	; Given a WordClassNode CLS and a WordNode WRD, alter the
	; counts on the disjuncts on WRD, so that they are orthogonal
	; to CLS.  If the counts are negative, that word-disjunct pair
	; is deleted (from the database as well as the atomspace).
	; The updated counts are stored in the database.
	;
	(define (orthogonalize CLS WRD)

		; Fold-helper to compute the dot-product between the WRD
		; vector and the CLS vector.
		(define (compute-dot-prod CLAPR DOT-PROD)
			(define cla (first CLAPR))
			(define wrd (second CLAPR))

			; The counts on each, or zero.
			(define cc (if (null? cla) 0 (LLOBJ 'pair-count cla)))
			(define wc (if (null? wrd) 0 (LLOBJ 'pair-count wrd)))

			(+ DOT-PROD (* cc wc))
		)

		; Compute the dot-product of WA and the merged class.
		(define dot-prod
			(fold compute-dot-prod 0.0 (ptu 'right-stars (list CLS WRD))))
		(define unit-prod (/ dot-prod lensq))

		; (format #t "sum ~A dot-prod=~A length=~A unit=~A\n"
		;      WRD dot-prod lensq unit-prod)

		; Alter the counts on the word so that they are orthogonal
		; to the class. Assumes that the dot-product was previously
		; computed, and also that the mean-square length of the
		; class was also previously computed.
		(define (ortho CLAPR)
			(define cla (first CLAPR))
			(define wrd (second CLAPR))

			; The counts on each, or zero.
			; Both cla and wrd are actually Sections.
			(define cc (if (null? cla) 0 (LLOBJ 'pair-count cla)))
			(define wc (if (null? wrd) 0 (LLOBJ 'pair-count wrd)))

			; The orthogonal component.
			(define orth (if (null? wrd) -999
					(- wc (* cc unit-prod))))

			; Update count on positive sections;
			; Delete non-positive sections. The deletion is not just
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

	(if (eq? 'WordNode (cog-type WA))
		(begin

			; Put the two words into the new word-class.
			(store-atom (MemberLink WA wrd-class))
			(store-atom (MemberLink WB wrd-class))

			(orthogonalize wrd-class WA)
			(orthogonalize wrd-class WB))

		; If WA is not a WordNode, assume its a WordClassNode.
		; The process is similar, but slightly altered.
		; We assume that WB is a WordNode, but perform no safety
		; checking to verify this.
		(begin
			; Add WB to the mrg-class (which is WA already)
			(store-atom (MemberLink WB wrd-class))

			; Redefine WB to be orthogonal to the word-class.
			(orthogonalize wrd-class WB))
	)
	wrd-class
)

; ---------------------------------------------------------------------

(define (merge-disambig COSOBJ COS-MIN WA WB)
"
  merge-disambig COS-MIN WA WB - merge WB into WA, returning the merged
  class.  If WA is a word, and not a class, then a new class is created
  and returned. The counts on both WA and WB are altered.

  COSOBJ is used to compute the cosine between WA and WB, and thus
     in needs to provide the 'right-cosine method.

  This is built on `merge-project`. See documentation for that.
"
	(define cosi (COSOBJ 'right-cosine WA WB))
	(define frac (/ (- cosi COS-MIN)  (- 1.0 COS-MIN)))
	(merge-project COSOBJ frac WA WB)
)

; ---------------------------------------------------------------
; stub wrapper for word-similarity.
; Return #t if the two should be merged, else return #f
; WORD-A might be a WordClassNode or a WordNode.
; XXX do something fancy here.
;
; COSOBJ must offer the 'right-cosine method
;
(define (ok-to-merge COSOBJ CUTOFF WORD-A WORD-B)

	(define (get-cosine) (COSOBJ 'right-cosine WORD-A WORD-B))

	(define (report-cosine)
		(let* (
; (foo (format #t "Start cosine ~A \"~A\" -- \"~A\"\n"
; (if (eq? 'WordNode (cog-type WORD-A)) "word" "class")
; (cog-name WORD-A) (cog-name WORD-B)))
				(start-time (get-internal-real-time))
				(sim (get-cosine))
				(now (get-internal-real-time))
				(elapsed-time (* 1.0e-9 (- now start-time))))

			(format #t "Cosine=~6F for ~A \"~A\" -- \"~A\" in ~5F secs\n"
				sim
				(if (eq? 'WordNode (cog-type WORD-A)) "word" "class")
				(cog-name WORD-A) (cog-name WORD-B)
				elapsed-time)
			(if (< CUTOFF sim) (display "------------------------------ Bingo!\n"))
			sim))

	; True, if cosine similarity is larger than the cutoff.
	(< CUTOFF (report-cosine))
)

; ---------------------------------------------------------------

(define (make-fuzz)
"
  make-fuzz -- Do fuzzy hard-coded merge.

  use `merge-project` with hard-coded frac=0.3 and min acceptable
  cosine=0.65
"
	(define cutoff 0.65)
	(define union-frac 0.3)

	(let* ((pca (make-pseudo-cset-api))
			(psa (add-dynamic-stars pca))
			(pcos (add-pair-cosine-compute psa))
		)
		(define (mpred WORD-A WORD-B)
			(ok-to-merge pcos cutoff WORD-A WORD-B))

		(define (merge WORD-A WORD-B)
			(merge-project pcos union-frac WORD-A WORD-B))

		; ------------------
		; Methods on this class.
		(lambda (message . args)
			(case message
				((merge-predicate)  (apply mpred args))
				((merge-function)   (apply merge args))
				(else               (apply pca (cons message args)))
			)))
)

; ---------------------------------------------------------------

(define (make-discrim)
"
  make-discrim -- Do a \"discriminating\" merge.

  use `merge-project` with sigmoid taper and
  hard-coded min acceptable cosine=0.50
"
	(define cutoff 0.50)

	(let* ((pca (make-pseudo-cset-api))
			(psa (add-dynamic-stars pca))
			(pcos (add-pair-cosine-compute psa))
		)
		(define (mpred WORD-A WORD-B)
			(ok-to-merge pcos cutoff WORD-A WORD-B))

		(define (merge WORD-A WORD-B)
			(merge-disambig pcos cutoff WORD-A WORD-B))

		; ------------------
		; Methods on this class.
		(lambda (message . args)
			(case message
				((merge-predicate)  (apply mpred args))
				((merge-function)   (apply merge args))
				(else               (apply pca (cons message args)))
			)))
)

; ---------------------------------------------------------------
; Example usage
;
; (load-atoms-of-type 'WordNode)          ; Typically about 80 seconds
; (define pca (make-pseudo-cset-api))
; (define psa (add-dynamic-stars pca))
;
; Verify that support is correctly computed.
; cit-vil is a vector of pairs for matching sections for "city" "village".
; Note that the null list '() means 'no such section'
;
; (define (bogus a b) (format #t "Its ~A and ~A\n" a b))
; (define ptu (add-tuple-math psa bogus))
; (define cit-vil (ptu 'right-stars (list (Word "city") (Word "village"))))
; (length cit-vil)
;
; Show the first three values of the vector:
; (ptu 'pair-count (car cit-vil))
; (ptu 'pair-count (cadr cit-vil))
; (ptu 'pair-count (caddr cit-vil))
;
; print the whole vector:
; (for-each (lambda (pr) (ptu 'pair-count pr)) cit-vil)
;
; Is it OK to merge?
; (define pcos (add-pair-cosine-compute psa))
; (ok-to-merge pcos (Word "run") (Word "jump"))
; (ok-to-merge pcos (Word "city") (Word "village"))
;
; Perform the actual merge
; (merge-project pcos 0.3 (Word "city") (Word "village"))
;
; Verify presence in the database:
; select count(*) from atoms where type=22;
