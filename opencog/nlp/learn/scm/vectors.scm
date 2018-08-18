;
; vectors.scm
;
; Specialized operations on vectors.
;
; Copyright (c) 2017, 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Vectors in the atomspace are inherently sparse (and you probably
; should not be using the atomspace to hold them, if they are not
; sparse.) The vector coefficients also tend to have a Zipfian
; distribution.  In many cases, dependencies between basis elements
; suggests that the atomspace data is more matroid-like, than
; vector-like.
;
; This makes the situation confusing: can data be treated like vectors,
; or not? The code here implements some vector-like operations, but
; with some strange twists.
;
; Currently, just one routine: vector addtion.
; XXX FIXME This should probably be moved to the atomspace/matrix
; directory.
;
;
; Combining vectors
; -----------------
; One way to "combine" two vectors is to perform simple vector addtion,
; summing them together component by component.
;
; Another plausbile combination strategy is to sum them, but only on the
; basis elements that have common support.  This is a kind-of
; "intersection" sum, because one takes the intersection of the sets of
; basis elements to find the common support.  For the non-shared basis
; elements, one might consider putting only a fraction of them into the
; final sum.
;
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
; wipes out those coefficients. Note that the total number of counts
; is preserved.  That is,
;
;   ||v_a|| + ||v_b|| = ||v_cluster|| + ||v_a^new|| + ||v_b^new||
;
; where ||v|| == ||v||_1 the l_1 norm aka count aka Manhattan-distance.
;
; If v_a and v_b have several word-senses in common; then so will
; v_cluster.  Since there is no a priori way to force v_a and v_b to
; encode only one common word sense, there needs to be some distinct
; mechanism to split v_cluster into multiple word senses, if that is
; needed.
;
; Union merging can be described using almost the same formulas, except
; that one takes
;
;   {e_union} = {e_a} set-union {e_b}
;
;
; Zipf Tails
; ----------
; The distribution of disjuncts on words is necessarily Zipfian. That
; is, the vectors could be called "Zipf vectors", in that the vector
; coefficients follow a Zipfian distribution.  There are many reasons
; why this is so, and multiple effects feed into this.
;
; It seems plausible to treat extremely-low frequency observations as
; a kind of noise, but which might also contain some signal. Thus,
; during merge, all of a Zipfian tail should be erged in. If its noise,
; it will tend to cancel during merge; if its signal, it will tend to be
; additive.
;
; That is, during merge, low-frequency observation counts should be
; merged in thier entirety, rather than split in parts, with one part
; remaining unmerged.  For example, if a word is to be merged into a
; word-class, and disjunct d has been observed 4 times or less, then
; all 4 of these observation counts should be merged into the word-class.
; Only high-frequency disjuncts can be considered to be well-known
; enough to be distinct, and thus suitable for fractional merging.
;
;
; merge-project
; -------------
; The above merge methods are implemented in the `merge-project`
; function. It takes, as an argument, a fractional weight which is
; used when the disjunct isn't shared between both words. Setting
; the weight to zero gives overlap merging; setting it to one gives
; union merging. Setting it to fractional values provides a merge
; that is intermediate between the two: an overlap, plus a bit more,
; viz some of the union.  A second parameter serves as a cutoff, so
; that any oservation counts below the cutoff are always merged.
;
; That is, the merger is given by the vector
;
;   v_merged = v_overlap + FRAC * (v_union - v_overlap)
;
; for those vector components in v_union that have been observed more
; than the minimum cutoff; else all of the small v_union components
; are merged.
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
; particular, the total number of observation counts in the system is
; not preserved (which is surely a bad thing, since counts are
; interpreted as probablity-frequencies.)
;
; There's no good reason for choosing `merge-ortho` over `merge-project`,
; and the broken probabilites is a good reason to reject `merge-ortho`.
; It is currently here for historical reasons -- it got coded funky, and
; that's that.  It should probably be eventually removed, when
; experimentation is done with.
;
;
; merge-discrim
; -------------
; Built on the merge-project method, the FRAC is a sigmoid function,
; ranging from 0.0 to 1.0, depending on the cosine between the vectors.
; The idea is that, if two words are already extremely similar, we may
; as well assume they really are in the same class, and so do a union
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
; * Fuzz: use `merge-project` with hard-coded frac=0.3 and cosine
;   distance with min acceptable cosine=0.65
;
; * Discrim: use `merge-discrim` with min acceptable cosine = 0.5
;
; * Info: use `merge-project` with hard-coded frac=0.3 and information
;   distance with min acceptable MI=3
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

(define-public (section-vec-sum WS LLOBJ FRAC ZIPF WA WB)
"
  section-vec-sum WS LLOBJ FRAC ZIPF WA WB - sum the vectors WA and
  WB, by summing the counts on SectionLinks anchored by WA and WB.
  Place the sums on Sections anchored by the atom WS. This does NOT
  alter the counts on WA and WB.

  WS can be any atom; it probably should NOT be a WordNode!
  WA and WB can be any atoms identifying Sections; they are typically
     WordNodes or possibly WordClassNodes.
  FRAC should be a floating point number between zero and one,
     indicating the fraction of a non-shared count to be used.
     Setting this to 1.0 gives the sum of the union of supports;
     setting this to 0.0 gives the sum of the intersection of supports.
  ZIPF is the smallest observation count, below which counts
     will not be divided up.
  LLOBJ is used to access counts on pairs.  Pairs are SectionLinks,
     that is, are (left-atom,right-atom) pairs wrapped in a SectionLink.
     Typically, left-Atom is a WordNode, and right-atom is a disjunct.

  The sum of the counts on Sections identified by WA and WB are
  performed. That is for each matching section on WA and WB, the counts
  are accessed, summed, and a corresponding Section is created on WS,
  with the count placed on that Section.

  The counts are summed in full only if both counts are non-zero.
  Otherwise, only a FRAC fraction of a single, unmatched count is
  copied to the corresponding Section on WS.

  The counts on WA and WB are NOT altered! This means that the total
  number of counts in the system is increased by this call.  This 
  implies that there can be a subtle corrpution of frequencies, if the
  WS atom is not chosen wisely -- e.g. if the WS is the same type as
  WA and WB (and WS is then stored to disk, or otherwise mixed in.)
"
	(define (bogus a b) (format #t "Sum ~A and ~A\n" a b))
	(define ptu (add-tuple-math LLOBJ bogus))

	; set-count ATOM CNT - Set the raw observational count on ATOM.
	(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 1 0 CNT)))

	; Sum two sections, placing the result ont WS.  Given a pair of
	; sections, sum the counts from each, and then place that count on a
	; corresponding section on WS.
	;
	; One or the other sections can be null. If both sections are not
	; null, then both are assumed to have exactly the same disjunct.
	;
	(define (sum-section-pair SECT-PAIR)
		; The two sections to merge
		(define lsec (first SECT-PAIR))
		(define rsec (second SECT-PAIR))

		; The counts on each, or zero.
		(define lcnt (if (null? lsec) 0 (LLOBJ 'get-count lsec)))
		(define rcnt (if (null? rsec) 0 (LLOBJ 'get-count rsec)))

		; If the other count is zero, take only a FRAC of the count.
		; But only if we are merging in a word, not a word-class;
		; we never want to shrink the support of a word-class, here.
		(define wlc (if (and (null? rsec) (< ZIPF lcnt)) (* FRAC lcnt) lcnt))
		(define wrc (if (and (null? lsec) (< ZIPF rcnt)) (* FRAC rcnt) rcnt))

		; Sum them.
		(define cnt (+ wlc wrc))

		; The cnt can be zero, if FRAC is zero.  Do nothing in this case.
		(if (< 1.0e-10 cnt)
			(let* (
					; The disjunct. Both lsec and rsec have the same disjunct.
					(seq (if (null? lsec) (cog-outgoing-atom rsec 1)
							(cog-outgoing-atom lsec 1)))
					; The new section
					(base (Section WS seq))
				)

				; The summed counts
				(set-count base cnt)
			))
	)

	(for-each sum-section-pair (ptu 'right-stars (list WA WB)))
)

; ---------------------------------------------------------------------
