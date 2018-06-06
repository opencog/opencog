;
; gram-blocks.scm
;
; Merge words into grammatical categories. Diagonal-block algo.
;
; Copyright (c) 2017, 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; See `gram-class.scm` for most of the documentation.  This performs
; classification (agglomerative clustering) of words into classes, but 
; using a block-diagonal algorithm that explores fewer words, in the
; hope of making more progress at the expense of lower accuracy.
;
; The main entry point here is the `chunk-over-words` routine.
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix) (opencog persist))

; ---------------------------------------------------------------
; Given a word or a grammatical class, and a list of words, scan
; the word list to see if any of them can be merged into the given
; word/class.  If so, then perform the merge, and return the
; word-class; else return the original word. If the initial merge
; can be performed, then the remainder of the list is scanned, to
; see if the word-class can be further enlarged.
;
; This is an O(n) algo in the length of WRD-LST.
;
; This is similar to `assign-word-to-class` function, except that
; the roles of the arguments are reversed, and this function tries
; to maximally expand the resulting class.
;
; WRD-OR-CLS should be the WordNode or WordClassNode to merge into.
; WRD-LST should be list of WordNodes to merge into WRD-OR-CLS.
; LLOBJ is the object to use for obtaining counts.
; FRAC is the fraction of union vs. intersection during merge.
; (These last two are passed blindly to the merge function).
;
(define (assign-expand-class LLOBJ FRAC WRD-OR-CLS WRD-LST)
	(if (null? WRD-LST) WRD-OR-CLS
		(let ((wrd (car WRD-LST))
				(rest (cdr WRD-LST)))
			; If the word can be merged into a class, then do it,
			; and then expand the class. Else try again.
			(if (ok-to-merge LLOBJ WRD-OR-CLS wrd)
				; Merge, and try to expand.
				(assign-expand-class LLOBJ FRAC
					(merge-ortho LLOBJ FRAC WRD-OR-CLS wrd) rest)
				; Else keep trying.
				(assign-expand-class LLOBJ FRAC WRD-OR-CLS rest))))
)

; ---------------------------------------------------------------
; Given a word-list and a list of grammatical classes, assign
; each word to one of the classes, or, if the word cannot be
; assigned, treat it as if it were a new class. Return a list
; of all of the classes, the ones that were given plus the ones
; that were created.
;
; A common use is to call this with an empty class-list, initially.
; In this case, words are compared pair-wise to see if they can be
; merged together, for a run-time of O(N^2) in the length N of WRD-LST.
;
; If CLS-LST is not empty, and is of length M, then the runtime will
; be roughly O(MN) + O(K^2) where K is what's left of the initial N
; words that have not been assigned to classes.
;
; If the class-list contains WordNodes (instead of the expected
; WordClassNodes) and a merge is possible, then that WordNode will
; be merged to create a class.
;
(define (block-assign-to-classes LLOBJ FRAC WRD-LST CLS-LST)
	(format #t "-------  Words remaining=~A Classes=~A ~A ------\n"
		(length WRD-LST) (length CLS-LST)
		(strftime "%c" (localtime (current-time))))

	; If the WRD-LST is empty, we are done; otherwise compute.
	(if (null? WRD-LST) CLS-LST
		(let* ((wrd (car WRD-LST))
				(rest (cdr WRD-LST))
				; Can we assign the word to a class?
				(cls (assign-word-to-class LLOBJ FRAC wrd CLS-LST)))

			; If the word was merged into an existing class, then recurse
			(if (eq? 'WordClassNode (cog-type cls))
				(block-assign-to-classes LLOBJ FRAC rest CLS-LST)

				; If the word was not assigned to an existing class,
				; see if it can be merged with any of the other words
				; in the word-list.
				(let* ((new-cls (assign-expand-class LLOBJ FRAC wrd rest))
						(new-lst
							(if (eq? 'WordClassNode (cog-type new-cls))
								; Use append, not cons, so as to preferentially
								; choose the older classes, as opposed to the
								; newer ones.
								; (cons new-cls CLS-LST)
								(append! CLS-LST (list new-cls))
								; else the old class-list
								CLS-LST)))
					(block-assign-to-classes LLOBJ FRAC rest new-lst)))))
)


; ---------------------------------------------------------------
; Loop over blocks of words, attempting to place them into grammatical
; classes. This is an O(N^2) algorithm, and so several "cheats" are
; employed to maintain some reasonable amount of forward progress. So,
;
; A) The list of words is ranked by order of the number of observations;
;    thus punctuation and words like "the", "a" come first.
; B) The ranked list is divided into power-of-two ranges, and only
;    the words in a given range are compared to one-another.
;
; The idea is that it is unlikely that words with very different
; observational counts will be similar.  NOTE: this idea has NOT
; been empirically tested, yet.
;
; TODO - the word-class list should probably also be ranked, so
; we preferentially add to the largest existing classes.
;
; XXX There are two user-adjustable parameters used below, to
; control the ranking. They should be exposed in the API or
; something like that! min-obs-cutoff, chunk-block-size
;
(define (chunk-over-words LLOBJ FRAC WRD-LST CLS-LST)
	; XXX Adjust the minimum cutoff as desired!!!
	; This is a tunable paramter!
	; Right now, set to 20 observations, minimum. Less
	; than this and weird typos and stuff get in.
	(define min-obs-cutoff 20)
	(define all-ranked-words (trim-and-rank LLOBJ WRD-LST min-obs-cutoff))

	; Been there, done that; drop the top-20.
	; (define ranked-words (drop all-ranked-words 20))
	; (define ranked-words all-ranked-words)
	; Ad hoc restart point. If we already have N classes, we've
	; probably pounded the cosines of the first 2N words or so into
	; a bloody CPU-wasting pulp. Avoid wasting CPU any further.
	(define ranked-words (drop all-ranked-words (* 1.6 (length CLS-LST))))

	(define (chunk-blocks wlist size clist)
		(if (null? wlist) '()
			(let* ((wsz (length wlist))
					; the smaller of word-list and requested size.
					(minsz (if (< wsz size) wsz size))
					; the first block
					(chunk (take wlist minsz))
					; the remainder
					(rest (drop wlist minsz))
					; perform clustering
					(new-clist (block-assign-to-classes LLOBJ FRAC chunk clist)))
				; Recurse and do the next block.
				; XXX the block sizes are by powers of 2...
				; perhaps they should be something else?
				(chunk-blocks rest (* 2 size) new-clist)
			)
		)
	)

	; The initial chunk block-size.  This is a tunable parameter.
	; Perhaps it should be a random number, altered between runs?
	(define chunk-block-size 20)
	(format #t "Start classification of ~A (of ~A) words, chunksz=~A\n"
		(length ranked-words) (length WRD-LST) chunk-block-size)
	(chunk-blocks ranked-words chunk-block-size CLS-LST)
)

; ---------------------------------------------------------------
