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

;
; gram-class.scm
;
; Merge words into grammatical categories.
;
; Copyright (c) 2017, 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
;
;
; Agglomerative clustering
; ------------------------
; The de facto algorithm implemented here is agglomerative clustering.
; That is, each word is compared to each of the existing clusters, and
; if it is close enough, it is merged in.  If a word cannot be assigned
; to a cluster, it is treated as a new cluster-point, and is tacked onto
; the list of existing clusters.
;
; That is, the existing clusters act as a sieve: new words either fall
; into one of the existing "holes", or start a new "hole".
;
; Note that clustering is an O(N^2) algrothm in the length N of the list
; of words: sooner or later, each word is effectively compared to every
; other word.  This has a disasterous impact on run-times, for large
; word lists.
;
; There are three variants of agglomerative clustering implemented in
; the code:
; * `loop-over-words` / `assign-to-classes`, which performs the above
;   algo, just as described.
; * `classify-pair-wise`, which is similar, except that, upon creating
;   a new cluster, it scans the entire word-list, attempting to add to
;   it. It is a very effective algorithm, but perhaps a bit
;   pathological, when the length of the word-list is long.
; * `chunck-over-words`, which is similar to `classify-pair-wise`,
;   except that it explores only a sequence of block diagonals down
;   the middle. Specifically, it examines the block of 20x20 pairs
;   of the most common words, follwed by the block 40x40 of the
;   pairs of the next-most-common 40 words, and then an 80x80 block...
;   The idea here is that similar words occur with similar frequencies,
;   e.g. punctuation (very high frequency) will get merged with other
;   punctuation, while common nouns (low frequency) get merged with
;   other common nouns.  An important issue with this variant is that
;   many word-pairs are not explored, although cross-frequency compares
;   can be forced by restarting the algo.
;
; Its hard to say which of these strategies is the "best". Both
; `loop-over-words` and `chunck-over-words` have desirable behaviors.
;
; One restart, all three algos suffer from a common problem: they create
; an ordered list of words by frequency; however, this is computed from
; the marginals for the words, which are no longer accurate.  Thus, the
; marginals (holding frequencies) should be recomputed before each
; restart.  This is not done automatically.
;
;
; ---------------------------------------------------------------
; Given a single word and a list of grammatical classes, attempt to
; assign the the word to one of the classes.
;
; Given a single word and a list of words, attempt to merge the word
; with one of the other words.
;
; In either case, return the class it was merged into, or just the
; original word itself, if it was not assigned to any of them.
; A core assumption here is that the word can be assigned to just one
; and only one class; thus, all merge determinations can be done in
; parallel.
;
; Run-time is O(n) in the length n of CLS-LST, as the word is
; compared to every element in the CLS-LST.
;
; See also the `assign-expand-class` function.
;
; WORD should be the WordNode to test.
; CLS-LST should be list of WordNodes or WordClassNodes to compare
;          to the WORD.
; LLOBJ is the object to use for obtaining counts.
; FRAC is the fraction of union vs. intersection during merge.
; (These last two are passed blindly to the merge function).
;
(define (assign-word-to-class LLOBJ FRAC WRD CLS-LST)

	; Return #t if cls can be merged with WRD
	(define (merge-pred cls) (ok-to-merge LLOBJ cls WRD))

	(let (; (cls (find merge-pred CLS-LST))
			(cls (par-find merge-pred CLS-LST))
		)
		(if (not cls)
			WRD
			(merge-ortho LLOBJ FRAC cls WRD)))
)

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
	(define ranked-words (drop all-ranked-words
			(inexact->exact (round (* 1.6 (length CLS-LST))))))

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
; Given a list of words, compare them pair-wise to find a similar
; pair. Merge these to form a grammatical class, and then try to
; expand that class as much as possible. Repeat until all pairs
; have been explored.  This is an O(N^2) algo in the length of the
; word-list!
;
; WRD-LST is the list of words to classify.
; GLST is a list of previously-determined classes.
;
; This returns a list of the classes that were created.
;
; This is a simpler version of `assign-to-classes`. It behaves
; differently; the attempt to "maximally expand" a new-found class
; means it will try to scan the entire word list, which is undesirable
; if the word-list is long.  Thus, this subroutine is currently not
; recommended, although its simple and easy, thus good for sanity
; checking.
;
(define (classify-pair-wise LLOBJ FRAC WRD-LST GLST)

	(define (check-pair WORD-A WORD-B CLS-LST)
		(if (ok-to-merge LLOBJ WORD-A WORD-B)
			(let ((grm-class (merge-ortho LLOBJ FRAC WORD-A WORD-B)))
				(assign-expand-class LLOBJ FRAC grm-class WRD-LST)
				(cons grm-class CLS-LST))))

	(fold-unordered-pairs GLST check-pair WRD-LST)
)
;
; ---------------------------------------------------------------
;
; Given a word-list and a list of grammatical classes, assign
; each word to one of the classes, or, if the word cannot be
; assigned, treat it as if it were a new class. Return a list
; of all of the classes, the ones that were given plus the ones
; that were created.
;
; WRD-LST is the list of words to be assigned to classes.
;
; TRUE-CLS-LST is a list of word-classes that words might possibley
;     get assigned to. This list should consist of WordClassNodes.
;     It can be initially empty; pairs of words than can be merged,
;     will be, to start a new class.
;
; FAKE-CLS-LIST is a list of singleton word-classes: pseudo-classes
;     that have only a single word in them. The list itself must
;     consist of WordNodes. It can be initially empty; if a word
;     cannot be merged into any existing class, then it will start
;     a new singleton class.
;
; The runtime is approximately O(N^2) + O(TN) + O(FN) where
;     N == (length WRD-LST)
;     T == (length TRUE-CLS-LST)
;     F == (length FAKE-CLS-LST)
;
; Currently, typical runtimes are about 1 second per pair, or about
; 0.5*500*500 = 35 hours for 500 words. This is NOT fast.
;
(define (assign-to-classes LLOBJ FRAC TRUE-CLS-LST FAKE-CLS-LST WRD-LST)
	(format #t "----  To-do =~A num-clases=~A num-done=~A ~A ----\n"
		(length WRD-LST) (length TRUE-CLS-LST) (length FAKE-CLS-LST)
		(strftime "%c" (localtime (current-time))))

	; If the WRD-LST is empty, we are done; otherwise compute.
	(if (null? WRD-LST) TRUE-CLS-LST
		(let* ((wrd (car WRD-LST))
				(rest (cdr WRD-LST))
				; Can we assign the word to a class?
				(cls (assign-word-to-class LLOBJ FRAC wrd TRUE-CLS-LST)))

			; If the word was merged into an existing class, then recurse
			(if (eq? 'WordClassNode (cog-type cls))
				(assign-to-classes LLOBJ FRAC TRUE-CLS-LST FAKE-CLS-LST rest)

				; If the word was not assigned to an existing class,
				; see if it can be merged with any of the singlton
				; words in the "fake-class" list.
				(let* ((new-cls (assign-word-to-class LLOBJ FRAC wrd FAKE-CLS-LST))
						(is-new-cls (eq? 'WordClassNode (cog-type new-cls)))
						(new-true
							(if is-new-cls
								; Use append, not cons, so as to preferentially
								; choose the older classes, as opposed to the
								; newer ones.
								(append! TRUE-CLS-LST (list new-cls))
								; else the true class list doesn't change
								TRUE-CLS-LST))
						(new-fake
							(if is-new-cls
								FAKE-CLS-LST
								; if its just a word, append it to the fake list
								(append! FAKE-CLS-LST (list new-cls)))))
					(assign-to-classes LLOBJ FRAC new-true new-fake rest)))))
)

; ---------------------------------------------------------------
; Given the list LST of atoms, trim it, discarding atoms with
; low observation counts, and then sort it, returning the sorted
; list, ranked in order of the observed number of sections on the
; word. (i.e. the sum of the counts on each of the sections).
;
; Words with fewer than MIN-CNT observations on them are discarded.
;
; Note: an earlier version of this ranked by the number of times
; each word was observed: viz:
;      (> (get-count ATOM-A) (get-count ATOM-B))
; However, for WordNodes, this does not work very well, as the
; observation count may be high from any-pair parsing, but
; infrequently used in MST parsing.
;
; The current version gets observation counts from the partial sums
; on the LLOBJ.  This is fine when starting from scratch, but gets
; distorted, as word-merges transfer counts from the word to the
; word-class, but fail to update the partial sums. XXX this needs
; fixing. XXX FIXME
;
(define (trim-and-rank LLOBJ LST MIN-CNT)
	(define pss (add-support-api LLOBJ))

	; nobs == number of observations
	(define (nobs WRD) (pss 'right-count WRD))

	; The support API won't work, if we don't have the wild-cards
	; in the atomspace before we sort. The wild-cards hold/contain
	; the support subtotals.
	(define start-time (get-internal-real-time))
	(for-each
		(lambda (WRD) (fetch-atom (LLOBJ 'right-wildcard WRD)))
		LST)

	(format #t "Finished fetching wildcards in ~5F sconds\n"
		(* 1.0e-9 (- (get-internal-real-time) start-time)))
	(format #t "Now trim to min of ~A observation counts\n" MIN-CNT)
	(sort!
		; Before sorting, trim the list, discarding words with
		; low counts.
		(filter (lambda (WRD) (<= MIN-CNT (nobs WRD))) LST)
		; Rank so that the highest support words are first in the list.
		(lambda (ATOM-A ATOM-B) (> (nobs ATOM-A) (nobs ATOM-B))))
)

; ---------------------------------------------------------------
; Loop over all words, attempting to place them into grammatical
; classes. This is an O(N^2) algorithm.
;
; The list of words is ranked by order of the number of observations;
; thus punctuation and words like "the", "a" come first.
;
; TODO - the word-class list should probably also be ranked, so
; we preferentially add to the largest existing classes.
;
; XXX There is a user-adjustable parameter used below, to
; control the ranking. It should be exposed in the API or
; something like that! min-obs-cutoff
;
(define (loop-over-words LLOBJ FRAC WRD-LST CLS-LST)
	; XXX Adjust the minimum cutoff as desired!!!
	; This is a tunable paramter!
	; Right now, set to 20 observations, minimum. Less
	; than this and weird typos and stuff get in.
	(define min-obs-cutoff 20)
	(define all-ranked-words (trim-and-rank LLOBJ WRD-LST min-obs-cutoff))

	; Ad hoc restart point. If we already have N classes, we've
	; surely pounded the cosines of the first 2N or so words into
	; a bloody CPU-wasting pulp. Avoid wasting CPU any further.
	(define ranked-words (drop all-ranked-words
			(inexact->exact (round (* 1.6 (length CLS-LST))))))

	(format #t "Start classification of ~A words\n"
		(length ranked-words))
	(assign-to-classes LLOBJ FRAC CLS-LST '() ranked-words)
)

; ---------------------------------------------------------------
;
; XXX FIXME the 0.3 is a user-tunable parameter, for how much of the
; non-overlapping fraction to bring forwards.
(define-public (gram-classify)
	(let* ((pca (make-pseudo-cset-api))
			(psa (add-dynamic-stars pca))
			(pcos (add-pair-cosine-compute psa))
			(start-time (get-internal-real-time))
		)

		(display "Start loading words and word-classes\n")
		(load-atoms-of-type 'WordNode)
		(load-atoms-of-type 'WordClassNode)
		; Verify that words have been loaded
		;  (define all-words (get-all-cset-words))
		; (define all-words (cog-get-atoms 'WordNode))
		(format #t "Finished loading ~A words in ~5f seconds\n"
			(length (cog-get-atoms 'WordNode))
			(* 1.0e-9 (- (get-internal-real-time) start-time)))

		; Attempt to merge words into wor-classes.
		; Stubbed out, because it has a poor performance profile.
		; Also, fails to make use of existing classes.
		; (classify-pair-wise pcos 0.3
		;	(cog-get-atoms 'WordNode)
		;	(cog-get-atoms 'WordClassNode))

		; Chunk over words might be faster, but is probably less
		; accurate. Use loop-ver-words in production.
		; (chunk-over-words pcos 0.3
		; 	(cog-get-atoms 'WordNode)
		; 	(cog-get-atoms 'WordClassNode))

		; Use this one.
		(loop-over-words pcos 0.3
			(cog-get-atoms 'WordNode)
			(cog-get-atoms 'WordClassNode))
	)
)

; ---------------------------------------------------------------
; Example usage
;
; (load-atoms-of-type 'WordNode)          ; Typicaly about 80 seconds
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
; (merge-ortho pcos 0.3 (Word "city") (Word "village"))
;
; Verify presence in the database:
; select count(*) from atoms where type=22;
