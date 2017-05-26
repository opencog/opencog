;
; pair-fold-api.scm
;
; Provides support for folding down tuples of pairs.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; There is sometimes a need to take differences and sums of vectors,
; or apply other kinds of transforms.  This provides eh API to do that.
;
; So, for example, the add-pair-support-compute object adds methods
; to compute the length of a vector (the l_2 norm) or more generally
; the l_p norm.  Suppose, instead, we wanted to take the length of
; the difference of two vectors? That is, length(A-B)? How can we do
; that?  This class provides a generic way of doing this.
;
; Suppose that some pair object is providing the vectors. Any of
; the classes make-pseudo-cset-api, make-any-link-api,
; make-clique-pair-api, etc. will do. To take the difference
; of two vectors, define the function
;
;     (define (subtract TUPLE)  (- (first TUPLE) (second TUPLE)))
;
; which just takes the numeric difference of a list of two numbers.
; For example: (subtract (list 7 2)) returns 5.  This can be used to
; subtract vectors, like so:
;
;      (define vecty (make-pseudo-cset-api)
;      (define subby (add-tuple-math vecty subtract))
;      (define normy (add-pair-support-compute subby))
;
; The length of the difference of two vectors can then be computed as
; so:
;      (normy 'right-length (list (Word "the") (Word "a")))
;
; which will take the disjunct-vectors for the two words "the" and "a",
; treating each disjunct as a basis element, take thier difference, and
; return the length (the root-mean-square of the difference of the
; counts).
;
; There are other possibilities.  To compute the number of disjuncts
; that these two words have in common, define a set-intersection
; function:
;
;    (define (intersect TUPLE)
;       (if (and (< 0 (abs (first TUPLE))) (< 0 (abs (second TUPLE)))) 1 0))
;
; then
;
;      (define isect (add-tuple-math vecty intersect))
;      (define secty (add-pair-support-compute isect))
;      (secty 'right-count (list (Word "the") (Word "a")))
;
; will return how many disjuncts there are shared, in common, with both
; of these words.  Similarly, the set union will count how many
; disjuncts are used, between the two:
;
;    (define (union TUPLE)
;       (if (or (< 0 (abs (first TUPLE))) (< 0 (abs (second TUPLE)))) 1 0))
;
; then
;
;      (define youny (add-tuple-math vecty union))
;      (define uniny (add-pair-support-compute youny))
;      (uniny 'right-count (list (Word "the") (Word "a")))
;
; returns the number of disjuncts that appear in the word "the" or in
; the word "a".
;
; The function provided to add-tuple-math can be any tuple. i.e. can
; take any arbitrary list.  The only constraint is that all elements of
; the list must all be items associated with the same vector.
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (ice-9 optargs)) ; for define*-public

; ---------------------------------------------------------------------
;
(define*-public (add-tuple-math LLOBJ FUNC
	#:optional (GET-CNT (lambda (x) (LLOBJ 'pair-count x))))
"
  add-tuple-math LLOBJ FUNC - Extend LLOBJ with ability to take
  tuples of items, and then call FUNC on that tuple, whenever
  the 'pair-count method is invoked.
"
	(let ((llobj LLOBJ)
			(star-obj (add-pair-stars LLOBJ))
			(sum-func FUNC)
			(get-cnt GET-CNT)
		)

		; ---------------
		; Return the set-union of all atoms that might be paired
		; with one of the atoms from TUPLE on the right.
		(define (get-left-union TUPLE)
			(delete-duplicates!
				(append-map!
					(lambda (item) (map! gar (star-obj 'left-stars item)))
					TUPLE)))

		(define (get-right-union TUPLE)
			(delete-duplicates!
				(append-map!
					(lambda (item) (map! gdr (star-obj 'right-stars item)))
					TUPLE)))

		; ---------------
		; Given a TUPLE of items of 'right-type, this returns
		; a tuple of low-level pairs of LEFTY and each righty
		; in the TUPLE.  If such a pair does not exist, the
		; returned tuple will contain an empty list at that locus.
		(define (get-left-lopr-tuple LEFTY TUPLE)
			(define prty (llobj 'pair-type))
			(map
				(lambda (rght) (cog-link prty LEFTY rght))
				TUPLE))

		(define (get-right-lopr-tuple RIGHTY TUPLE)
			(define prty (llobj 'pair-type))
			(map
				(lambda (left) (cog-link prty left RIGHTY))
				TUPLE))

		; ---------------
		; Expects TUPLE to be a scheme list of items of 'right-type.
		; Returns a list of tuples of the left-stars appropriae for
		; that TUPLE.  The left-star tuples are "aligned", so that
		; that within one tuple, all pairs have exactly the same
		; left side.  If such a pair does not exist, the empty list
		; stands in its place.  The only case where this would not
		; happen is if all items in the TUPLE had exactly the same
		; left-wilds.  But this would be a very unusual thing, in the
		; normal case.
		;
		; So, for example, if TUPLE is (list (Word "the") (Word "a"))
		; This might return a list of say, 3 tuples:
		;   (list
		;      (list  ; Note left atoms are identical.
		;          (ListLink (Word "foo") (Word "the"))
		;          (ListLink (Word "foo") (Word "a")))
		;      (list
		;          '()  ; Note the pair bar:the does not exist
		;          (ListLink (Word "bar") (Word "a")))
		;      (list
		;          (ListLink (Word "zed") (Word "the"))
		;          '()))  ; Note the par zed-a does not exist.
		;
		; In the above, the union of the left support was {foo, bar, zed}
		; and the intersection of the left support was just {foo}.  This
		; will be the typical case: the intersection will be typically
		; non-empty, and hethe union will typically be strictly alrger.

		(define (left-star-union TUPLE)
			(map
				(lambda (lefty) (get-left-lopr-tuple lefty TUPLE))
				(get-left-union TUPLE)))

		; Same as above, but for the right
		(define (right-star-union TUPLE)
			(map
				(lambda (righty) (get-right-lopr-tuple righty TUPLE))
				(get-right-union TUPLE)))

		; ---------------
		; Given a TUPLE of low-level pairs, return a tuple of high-level
		; pairs.
		(define (get-pair TUPLE)
			(map (lambda (lopr) (llobj 'item-pair lopr)) TUPLE))

		; Given a TUPLE of high-level pairs, return a single number.
		; The sum-func is applied to reduce the counts on each pair
		; in the tuple down to just one number.
		(define (get-func-count TUPLE)
			(sum-func
				(map
					(lambda (pr) (if (null? pr) 0 (get-cnt pr)))
					TUPLE)))

		; ---------------
		; Return a pointer to each method that this class overloads.
		(define (provides meth)
			(case meth
				((left-stars) left-star-union)
				((right-stars) right-star-union)
				((item-pair) get-pair)
				((pair-count) get-func-count)
				(else #f)))

		; ---------------

	; Methods on this class.
	(lambda (message . args)
		(case message
			((left-stars)      (apply left-star-union args))
			((right-stars)     (apply right-star-union args))
			((item-pair)       (apply get-pair args))
			((pair-count)      (apply get-func-count args))
			((provides)        (apply provides args))
			(else (apply llobj (cons message args))))
		)))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
