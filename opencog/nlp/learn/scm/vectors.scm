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
; Definitions
; -----------
; This section provides some formal defintions for computing the
; intersection of basis elements of vectors.
; Let
;
;   v_a, v_b be two vectors. The coefficients of each basis element
;         is assumed to be obtainable from the 'count method.
;   {e_a} = set of basis elements in v_a with non-zero coefficients
;   {e_b} = set of basis elements in v_b with non-zero coefficients
;   {e_overlap} = {e_a} set-intersection {e_b}
;   {e_union} = {e_a} set-union {e_b}
;   pi_overlap = unit on diagonal for each e in {e_overlap}
;              == projection matrix onto the subspace {e_overlap}
;   v_a^pi = pi_overlap . v_a == projection of v_a onto {e_overlap}
;   v_b^pi = pi_overlap . v_b == projection of v_b onto {e_overlap}
;
;   pi_perp = projection matrix for the rest of the vector space,
;             so that pi_perp \oplus pi_overlap = identity matrix.
;   v_a^perp = v_a - v_a^pi
;   v_b^perp = v_b - v_b^pi
;
;   v^overlap = v_a^pi + v_b^pi
;   v^union(beta) = v_overlap + beta (v_a^perp + v_b^perp)
;
; Here, beta is a constant, typically ranging from zero to one. For
; beta==1, v^union becomes just ordinary vector addition. That is,
;
;   v^union(1) = v_a + v_b
;
;
; Zipf Tails
; ----------
; In most practical cases, the distribution of the values of the vector
; coefficients is typically Zipfian, suggesting that they should be
; called "Zipf vectors", or possibly "Zipf matroids".
;
; It seems plausible to treat extremely-low frequency observations as
; a kind of noise, but which might also contain some signal. Thus,
; during vector combination, it seems to make sense to merge in all of
; a Zipfian tail. If its noise, it will tend to cancel during merge;
; if its signal, it will tend to be additive.
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
