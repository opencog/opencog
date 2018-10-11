;
; gram-agglo.scm
;
; Merge words into grammatical categories. Agglomerative clustering.
;
; Copyright (c) 2017, 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; When a pair of words are judged to be grammatically similar, they
; can be used to create a "grammatical class", containing both the
; words, and behaving as their average.  When a word is judged to
; belong to an existing grammatical-class, then some mechanism must
; be provided to add that word to the class.  This file implements
; the tools for creating and managing such classes.  It does not
; dictate how to judge when words belong to a class; this is done
; independently of the structure of the classes themselves.
;
; The above describes the general concept of "agglomerative clustering",
; which is what is effectively implemented in this file.  Note, however,
; that the general problem is not quite this simple: in addition to
; assigning words to grammatical classes, one must also cluster the
; connectors, which in turn alters the notion of similarity. That is,
; words are not isolated points to be clustered; the location of those
; "points" depend on the connectors and sections ("disjuncts") which
; must also be clustered in a consistent manner: these two clustering
; steps form a feedback loop.
;
;
; Agglomerative clustering
; ------------------------
; This file implements four different variants of agglomerative
; clustering. The variants differ according to the order in which
; they scan the word-lists to be clustered. Different scan policies
; can make a huge difference in performance.
;
; Two of the algorithms implemented in this file are effectively O(N^2)
; algorithms, in the length N of the list of words. One does a bit
; better, and one might(?) approach O(N log N) performance.
;
; The first three variants are these:
;
; * `agglo-over-words` / `assign-to-classes`, which performs a basic
;   sieving-style agglo: Each word is compared to each of the existing
;   clusters, and if it is close enough, it is merged in.  If a word
;   cannot be assigned to a cluster, it is treated as a new cluster-
;   point, and is tacked onto the list of existing clusters. That is,
;   the existing clusters act as a sieve: new words either fall into
;   one of the existing "holes", or start a new "hole".
;
; * `classify-pair-wise`, which is similar, except that, upon creating
;   a new cluster, it scans the entire word-list, attempting to add to
;   it. This scanning order makes it 'almost' hierarchical.  It behaves
;   a bit pathologically when the length of the word-list is long.
;
; * `diag-over-words`, which is similar to `classify-pair-wise`,
;   except that it explores only a sequence of block diagonals down
;   the middle. Specifically, it examines the block of 20x20 pairs
;   of the most common words, followed by the block 40x40 of the
;   pairs of the next-most-common 40 words, and then an 80x80 block...
;   The idea here is that similar words occur with similar frequencies,
;   e.g. punctuation (very high frequency) will get merged with other
;   punctuation, while common nouns (low frequency) get merged with
;   other common nouns.  An important issue with this variant is that
;   many word-pairs are not explored, although cross-frequency compares
;   can be forced by restarting the algo.
;
; One restart, all of the algos suffer from a common problem: they create
; an ordered list of words by frequency; however, this is computed from
; the marginals for the words, which are no longer accurate.  Thus, the
; marginals (holding frequencies) should be recomputed before each
; restart.  This is not done automatically.
;
;
; Current best strategy: `greedy-grow`
; ------------------------------------
; The current best merge strategy is implemented in `greedy-grow` and
; makes use of several tricks, based on practical experience:
;
; * Words are sorted into frequency order, thus giving a rough-cut of
;   having grammatically-similar words relatively near one-another in
;   the list.
; * Main loop is agglomerative: if a word can be assigned to an existing
;   word class, then it will be, and then processing moves to the next
;   word.
; * If a word cannot be assigned to an existing word-class, it is
;   designated to start a new 'singleton' class, of which it is the
;   only member. For assignment of of words to classes, the most populous
;   classes are considered first, and the singleton classes are
;   considered last.
; * When a new cluster is formed, then perform the "greedy"/maximal
;   cluster expansion, scanning much of the word list to try to grow
;   the cluster further. Key here is the phrase "much of": instead
;   of scanning the entire list (which has a lot of dregs at the end)
;   only scan the top M words, with M=max(200,9D) where D = number of
;   words scanned so far. Thus, excessive searching down into the
;   low-frequency boon-docks is avoided.
; * Once a word has been assigned to a cluster, its added to the 'done'
;   list.  Words in the 'done' list are re-scanned, every time that
;   a new cluster is created, to see if they might also fit into the
;   new cluster.  This is to allow words to have multiple meanings,
;   e.g. to be classified as both nouns and verbs. Just because a word
;   got classified as a noun in the first pass, does not mean that
;   it should be forgotten - it might also be a verb.
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix) (opencog persist))

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
; MERGER is the object that provides the merge predicate and function
;
(define (assign-word-to-class MERGER WRD CLS-LST)

	; Return #t if cls can be merged with WRD
	(define (merge-pred cls) (MERGER 'merge-predicate cls WRD))

	(let ((cls (find merge-pred CLS-LST))
			; Sigh. The parallel-find is actually slower than
			; the serial find.
			; (cls (par-find merge-pred CLS-LST))
		)
		(if (not cls)
			WRD
			(MERGER 'merge-function cls WRD)))
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
; MERGER is the object that provides the merge predicate and function.
;
(define (assign-expand-class MERGER WRD-OR-CLS WRD-LST)
	(if (null? WRD-LST) WRD-OR-CLS
		(let ((wrd (car WRD-LST))
				(rest (cdr WRD-LST)))
			; If the word can be merged into a class, then do it,
			; and then expand the class. Else try again.
			(if (MERGER 'merge-predicate WRD-OR-CLS wrd)
				; Merge, and try to expand.
				(assign-expand-class MERGER
					(MERGER 'merge-function WRD-OR-CLS wrd) rest)
				; Else keep trying.
				(assign-expand-class MERGER WRD-OR-CLS rest))))
)

; ---------------------------------------------------------------
; Sort the class list, returning a list of classes from the largest,
; to the smallest.
;
(define (sort-class-list CLS-LST)

	; Return an integer, the number of words in the class
	(define (nwords-in-cls CLS)
		(fold
			(lambda (MEMB sum)
				(if (eq? (cog-type (gar MEMB)) 'WordNode) (+ sum 1) sum))
			0
			(cog-incoming-by-type CLS 'MemberLink)))

	; Sort the class-list according to size.
	(sort! CLS-LST
		; Rank so that the highest counts are first in the list.
		(lambda (ATOM-A ATOM-B)
			(> (nwords-in-cls ATOM-A) (nwords-in-cls ATOM-B))))
)

; ---------------------------------------------------------------
; Utilities used by several of the functions

; Return true if WRD is in word-class CLS
(define (is-in-cls? WRD CLS)
	(not (null? (cog-link 'MemberLink WRD CLS))))

; Return a list of words that got placed into the class.
(define (got-done WRDS CLS)
	(filter (lambda (w) (is-in-cls? w CLS)) WRDS))

; Return a list of words NOT in the class.
(define (still-to-do WRDS CLS)
	(remove (lambda (w) (is-in-cls? w CLS)) WRDS))

; ---------------------------------------------------------------
; ---------------------------------------------------------------
; Various different agglomerative clustring algos follow below.
;
; These are the main, recursive work-horses; they need to be
; set up by initialization routines, before being usable.
; The initializers are farther down in the file.
; ---------------------------------------------------------------
;
; block-assign-to-classes -- cluster in in diagonal blocks.
;
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
(define (block-assign-to-classes MERGER WRD-LST CLS-LST)
	(format #t "-------  Words remaining=~A Classes=~A ~A ------\n"
		(length WRD-LST) (length CLS-LST)
		(strftime "%c" (localtime (current-time))))

	; If the WRD-LST is empty, we are done; otherwise compute.
	(if (null? WRD-LST) CLS-LST
		(let* ((wrd (car WRD-LST))
				(rest (cdr WRD-LST))
				; Can we assign the word to a class?
				(cls (assign-word-to-class MERGER wrd CLS-LST)))

			; If the word was merged into an existing class, then recurse
			(if (eq? 'WordClassNode (cog-type cls))
				(block-assign-to-classes MERGER rest CLS-LST)

				; If the word was not assigned to an existing class,
				; see if it can be merged with any of the other words
				; in the word-list.
				(let* ((new-cls (assign-expand-class MERGER wrd rest))
						(new-lst
							(if (eq? 'WordClassNode (cog-type new-cls))
								; Use append, not cons, so as to preferentially
								; choose the older classes, as opposed to the
								; newer ones.
								; (cons new-cls CLS-LST)
								(append! CLS-LST (list new-cls))
								; else the old class-list
								CLS-LST)))
					(block-assign-to-classes MERGER rest new-lst)))))
)

; ---------------------------------------------------------------
;
; assign-to-classes - Simple agglomerative clustering
;
; Given a word-list and a list of grammatical classes, assign
; each word to one of the classes, or, if the word cannot be
; assigned, treat it as if it were a new class. Return a list
; of all of the classes, the ones that were given plus the ones
; that were created.
;
; WRD-LST is the list of words to be assigned to classes.
;
; TRUE-CLS-LST is a list of word-classes that words might possibly
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
(define (assign-to-classes MERGER TRUE-CLS-LST FAKE-CLS-LST WRD-LST)

	(format #t "--- To-do=~A num-classes=~A num-done=~A ~A ---\n"
		(length WRD-LST) (length TRUE-CLS-LST) (length FAKE-CLS-LST)
		(strftime "%c" (localtime (current-time))))

	; If the WRD-LST is empty, we are done; otherwise compute.
	(if (null? WRD-LST) TRUE-CLS-LST
		(let* ((wrd (car WRD-LST))
				(rest (cdr WRD-LST))
				; Can we assign the word to a class?
				(cls (assign-word-to-class MERGER wrd TRUE-CLS-LST)))

			; If the word was merged into an existing class, then recurse
			(if (eq? 'WordClassNode (cog-type cls))
				(assign-to-classes MERGER TRUE-CLS-LST FAKE-CLS-LST rest)

				; If the word was not assigned to an existing class,
				; see if it can be merged with any of the singleton
				; words in the "fake-class" list.
				(let* ((new-cls (assign-word-to-class MERGER wrd FAKE-CLS-LST))
						(is-new-cls (eq? 'WordClassNode (cog-type new-cls)))
						(new-true
							(if is-new-cls
								(sort-class-list (cons new-cls TRUE-CLS-LST))
								; else the true class list doesn't change
								TRUE-CLS-LST))
						(new-fake
							(if is-new-cls
								; The new fake-list is now shorter
								(still-to-do FAKE-CLS-LST new-cls)

								; If its just a word, append it to the
								; fake list.  Use append, not cons, so as
								; to preferentially choose the older words,
								; as opposed to the newer ones.
								(append! FAKE-CLS-LST (list new-cls)))))
					(assign-to-classes MERGER new-true new-fake rest)))))
)

; ---------------------------------------------------------------
;
; greedy-grow -- Agglomerative clustering; grow new classes first
;
; Given a word-list and a list of grammatical classes, assign
; each word to one of the classes, or, if the word cannot be
; assigned, treat it as if it were a new class. Return a list
; of all of the classes, the ones that were given plus the ones
; that were created.
;
; Similar to `assign-to-classes`, except that this attempts to
; maximally grow a newly-created class. It also attempts to manage
; the word-list better, deferring reconsideration of recently-assigned
; words for a later pass.
;
; WRD-LST is the list of words to be assigned to classes.
;
; TRUE-CLS-LST is a list of word-classes that words might possibly
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
; DONE-LST is a list of words that have been placed into at least one
;     cluster; they will be passed over a second time, to see where
;     else they might fit. Viz, many words have both noun and verb
;     forms, and thus need to go into multiple classes.
;
; XXX FIXME: The DONE-LIST should be scrubbed for short junk. That is,
; words in the DONE-LIST have a good chance of being completely
; neutered, with almost nothing left in them. They should get dropped.
;
; XXX Maybe-FIXME: There's some amount of pointless recomputation of
; cosines between the word-list, and the existing grammatical classes.
; During the construction of the classes, a greedy search was formed
; part-way down the word-list. Thus, when resuming the search, its
; (mostly) pointless to compute the cosines between the uncategorized
; words, and the existing categories. Fixing this wastefulness is hard,
; because its hard to track how far down the list we went for each
; category. I just don't see anything except a super-complicated
; solution that does not seem worth the effort. So we'll let the
; CPU work harder than it might otherwise need to.
;
(define *-greedy-anchor-* (AnchorNode "*-greedy-singleton-words-*"))
(define (greedy-grow MERGER TRUE-CLS-LST FAKE-CLS-LST DONE-LST WRD-LST)

	; Tunable parameters
	(define min-greedy 200)
	(define scan-multiplier 4)

	; How many words have been classified?
	(define (num-classified-words)
		(define (nmemb CLS) (length (cog-incoming-by-type CLS 'MemberLink)))
			(fold (lambda (CLS cnt) (+ cnt (nmemb CLS))) 0 TRUE-CLS-LST))

	; How far to scan ahead?
	(define (num-to-scan)
		(max min-greedy (* scan-multiplier
			(+ (num-classified-words) (length FAKE-CLS-LST)))))

	; If only a stub of a word is left, throw it away.
	(define (keep WORD)
		(MERGER 'clobber)
		(format #t "---- Remaining count = ~6F for ~A"
			((add-support-compute MERGER) 'right-count WORD) WORD)
		(if (MERGER 'discard? WORD) '() (list WORD)))

	(format #t "--- To-do=~A ncls=~A sing=~A nredo=~A ~A -- \"~A\" ---\n"
		(length WRD-LST) (length TRUE-CLS-LST) (length FAKE-CLS-LST)
		(length DONE-LST)
		(strftime "%F %T" (localtime (current-time)))
		(if (null? WRD-LST) '() (cog-name (car WRD-LST)))
	)

	; If the WRD-LST is empty, we are done; otherwise compute.
	(if (null? WRD-LST) TRUE-CLS-LST
		(let* ((wrd (car WRD-LST))
				(rest (cdr WRD-LST))
				; Attempt to assign the word to an existing class.
				(cls (assign-word-to-class MERGER wrd TRUE-CLS-LST)))

			; If the word was merged into an existing class, then
			; recurse.  Place the word onto the done-list.
			(if (eq? 'WordClassNode (cog-type cls))
				(greedy-grow MERGER TRUE-CLS-LST FAKE-CLS-LST
					(append! DONE-LST (keep wrd)) rest)

				; If the word was not assigned to an existing class,
				; see if it can be merged with any of the singleton
				; words in the "fake-class" list.
				(let ((new-cls (assign-word-to-class MERGER wrd FAKE-CLS-LST)))

					; If we failed to create a new class, then just
					; append the word to the fake list, and recurse.
					; Store the fake list in the database, as well.
					; This is used in case of restart, and also for
					; tracking progress statistics.
					(if (eq? 'WordNode (cog-type new-cls))
						(begin
							(store-atom (Member new-cls *-greedy-anchor-*))
							(greedy-grow MERGER TRUE-CLS-LST
								(append! FAKE-CLS-LST (list new-cls))
								DONE-LST rest))

						; If the result is a new class, then greedy-grow it.
						; But only consider a limited subset of the word list,
						; so as to not blow out performance. See if any of
						; the previously-assigned words might also go into the
						; new class. And then recurse.
						(let* ((short-list (take rest
									(min (num-to-scan) (length rest)))))
							(format #t "--- Greedy-checking next ~A items\n"
								(min (num-to-scan) (length rest)))
							(assign-expand-class MERGER new-cls short-list)
							(format #t "--- Checking the done-list len=~A\n"
								(length DONE-LST))
							(assign-expand-class MERGER new-cls DONE-LST)

							; If anything was merged from the fake-list,
							; then remove it from the anchor as well.
							; (this is a database-delete).
							(for-each
								(lambda (unfake)
									(cog-delete (Member unfake *-greedy-anchor-*)))
								(got-done FAKE-CLS-LST new-cls))

							; If anything from the done-list was merged, then
							; what is left might be either a tiny piece of
							; crud, or it might be a something that can now
							; be merged into an existing class.  We should
							; handle this. XXX TODO.
							(greedy-grow MERGER
								; The new true-list is now longer.
								(sort-class-list (cons new-cls TRUE-CLS-LST))

								; The new fake-list is now shorter
								(still-to-do FAKE-CLS-LST new-cls)

								; The new done-list is probably a lot longer
								(append! DONE-LST
									(got-done FAKE-CLS-LST new-cls)
									(keep wrd)
									(got-done short-list new-cls))

								; The new todo list is probably a lot shorter
								(still-to-do rest new-cls))
						))))))
)

; ---------------------------------------------------------------
; ---------------------------------------------------------------
; ---------------------------------------------------------------
; Agglomerative clustering entry points.
;
; The functions below set up parameters for the work-horses above.
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
(define (classify-pair-wise MERGER WRD-LST GLST)

	(define (check-pair WORD-A WORD-B CLS-LST)
		(if (MERGER 'merge-predicate WORD-A WORD-B)
			(let ((grm-class (MERGER 'merge-function WORD-A WORD-B)))
				(assign-expand-class MERGER grm-class WRD-LST)
				(cons grm-class CLS-LST))))

	(format #t "Start pair-wise classification of ~A words\n"
		(length WRD-LST))
	(fold-unordered-pairs GLST check-pair WRD-LST)
)

; ---------------------------------------------------------------
; Loop over all words, attempting to place them into grammatical
; classes. This is an O(N^2) algorithm.
;
(define (agglo-over-words MERGER WRD-LST CLS-LST)

	(format #t "Start agglo classification of ~A words\n"
		(length WRD-LST))
	(assign-to-classes MERGER CLS-LST '() WRD-LST)
)

; ---------------------------------------------------------------
; Loop over blocks of words, attempting to place them into grammatical
; classes. This tries to be faster than an O(N^2) algorithm, by
; employing a cheat to get reasonable forward progress. Specifically:
;
; The ranked list is divided into power-of-two ranges, and only the
; words in a given range are compared to one-another.  The idea is
; that it is unlikely that words with very different observational
; counts will be similar.  NOTE: This idea has NOT been empirically
; measured, confirmed, tested, yet. It seems to be the case, but
; actual measurements have not been made.
;
; XXX There is a user-adjustable parameter used below, `diag-block-size`,
; to specify the initial block size.  This could be exposed in the API,
; maybe.  On the other hand, it could stay hard-coded forever, for all
; practical purposes.
;
(define (diag-over-words MERGER WRD-LST CLS-LST)

	(define (diag-blocks wlist size clist)
		(if (null? wlist) '()
			(let* ((wsz (length wlist))
					; the smaller of word-list and requested size.
					(minsz (if (< wsz size) wsz size))
					; the first block
					(chunk (take wlist minsz))
					; the remainder
					(rest (drop wlist minsz))
					; perform clustering
					(new-clist (block-assign-to-classes MERGER chunk clist)))
				; Recurse and do the next block.
				; XXX the block sizes are by powers of 2...
				; perhaps they should be something else?
				(diag-blocks rest (* 2 size) new-clist)
			)
		)
	)

	; The initial chunk block-size.  This is a tunable parameter.
	; Perhaps it should be a random number, altered between runs?
	(define diag-block-size 20)

	; Ad hoc restart point. If we already have N classes, we've
	; surely pounded the cosines of the first 2N or so words into
	; a bloody CPU-wasting pulp. Avoid wasting CPU any further.
	(define num-to-drop (inexact->exact (round (* 1.6 (length CLS-LST)))))
	(define ranked-words (drop WRD-LST num-to-drop))

	(format #t "Drop first ~A words from consideration, leaving ~A\n"
		num-to-drop (length ranked-words))
	(format #t "Start diag-block of ~A words, chunksz=~A\n"
		(length ranked-words) diag-block-size)
	(diag-blocks ranked-words diag-block-size CLS-LST)
)

; ---------------------------------------------------------------
; Loop over blocks of words, attempting to place them into grammatical
; classes. This attempts to be an O(N log N) algorithm, and so employs
; several tricks to try to get better than the naive O(N^2) perf.
;
; The idea is that it is unlikely that words with very different
; observational counts will be similar.  NOTE: this idea has NOT
; been empirically tested, yet.
;
(define (greedy-over-words MERGER WRD-LST CLS-LST)

	; Get the list of words that have been classified already.
	(define mdone-list
		(fold (lambda (CLS LST)
			(append! LST (map gar (cog-incoming-by-type CLS 'MemberLink))))
			'() CLS-LST))

	; Make sure that they really are words. (This should be a no-op...)
	(define done-list
		(filter! (lambda (w) (eq? 'WordNode (cog-type w))) mdone-list))

	(define (is-done? w)
		(find (lambda (x) (equal? x w)) done-list))

	; Trim the word-list, keeping only the not-done words.
	(define remain-words (remove! is-done? WRD-LST))

	; Fetch all of the singletons
	(define junk (fetch-incoming-by-type *-greedy-anchor-* 'MemberLink))

	(define singletons (map gar
		(cog-incoming-by-type *-greedy-anchor-* 'MemberLink)))

	(define (is-single? w)
		(find (lambda (x) (equal? x w)) singletons))

	(define todo-words (remove! is-single? remain-words))

	(format #t "Start greedy-agglomeration of ~A words\n"
		(length todo-words))
	(format #t "Existing classes=~A singletons=~A done=~A\n"
		(length CLS-LST) (length singletons) (length done-list))
	(greedy-grow MERGER CLS-LST singletons done-list todo-words)

	; XXX FIXME ... at the conclusion of this, we have a done list,
	; which, because of repeated merging, might possibly have been
	; reduced to single senses, which can now be classified.
)

; ---------------------------------------------------------------
; ---------------------------------------------------------------
; ---------------------------------------------------------------
; ---------------------------------------------------------------
; Main entry-point helpers.
;
(define (load-stuff)
	(define start-time (get-internal-real-time))

	(display "Start loading words and word-classes\n")
	(load-atoms-of-type 'WordNode)
	(load-atoms-of-type 'WordClassNode)
	(for-each
		(lambda (cls) (fetch-incoming-by-type cls 'MemberLink))
		(cog-get-atoms 'WordClassNode))

	; Verify that words have been loaded
	;  (define all-words (get-all-cset-words))
	; (define all-words (cog-get-atoms 'WordNode))
	(format #t "Finished loading ~A words in ~5f seconds\n"
		(length (cog-get-atoms 'WordNode))
		(* 1.0e-9 (- (get-internal-real-time) start-time)))
)

; Given the list WRD-LST of atoms, trim it, discarding atoms with
; low observation counts, and then sort it, returning the sorted
; list, ranked in order of the observed number of sections on the
; word. (i.e. the sum of the counts on each of the sections).
;
; Words with low observation counts are discarded. A word is
; considered to be 'big enough' if the LLOBJ says it is.
; Sorting is important, because of locality: words in similar
; grammatical classes also have similar frequency counts.
;
; Counts are obtained by looking them up in the margin. It is not
; practical, at this point, to count them directly, as this would
; require the entire matrix to be loaded.  Thus, only the marginal
; counts are referenced.  Its up to the clustering algos, later on, to
; verify counts, as desired. (The point here being that the marginal
; counts are going to be a bit off, since these counts are altered by
; mergers, rendering the cached marginal values incorrect.
;
(define (trim-and-rank LLOBJ WRD-LST)
	(define pss (add-support-api LLOBJ)) ; from the margins

	; nobs == number of observations
	(define (nobs WRD) (pss 'right-count WRD))

	; The support API won't work, if we don't have the wild-cards
	; in the atomspace before we sort. The wild-cards hold/contain
	; the support subtotals.
	(define start-time (get-internal-real-time))
	(for-each
		(lambda (WRD) (fetch-atom (LLOBJ 'right-wildcard WRD)))
		WRD-LST)

	(format #t "Finished fetching wildcards in ~5F seconds\n"
		(* 1.0e-9 (- (get-internal-real-time) start-time)))

	; Before sorting, trim the list, discarding words with low counts.
	(let* ((tr-start (get-internal-real-time))
			(trimed-words
				(remove (lambda (WRD) (LLOBJ 'discard-margin? WRD)) WRD-LST)))

		(format #t "Trimmed in ~5F seconds\n"
			(* 1.0e-9 (- (get-internal-real-time) tr-start)))

		(format #t "After triming, ~A words left, out of ~A\n"
			(length trimed-words) (length WRD-LST))

		(let* ((ra-start (get-internal-real-time))
				(ranked-words
					(sort! trimed-words
						; Rank so that the commonest words are first in the list.
						(lambda (ATOM-A ATOM-B) (> (nobs ATOM-A) (nobs ATOM-B))))))

			(format #t "Sorting in ~5F seconds\n"
				(* 1.0e-9 (- (get-internal-real-time) ra-start)))

			ranked-words))
)

; Attempt to merge words into word-classes.
;
; MERGER should be a lambda accepting two words, and returning #t or #f
; depending on whether the words should be merged together of not.
;
; MIN-OBS is the minumum number of observations that a word should have,
; in order to be considered for merging.
;
(define (gram-classify ALGO MERGER)
	(load-stuff)
	(let* ((wrd-lst (cog-get-atoms 'WordNode))
			(ranked-words (trim-and-rank MERGER wrd-lst))
			(cls-lst (cog-get-atoms 'WordClassNode))
			(sorted-cls (sort-class-list cls-lst)))
		(ALGO MERGER ranked-words sorted-cls))
)

; ---------------------------------------------------------------
; ---------------------------------------------------------------
; Main entry points for word-classification,
;
(define-public (gram-classify-pair-wise COS-CUT FRAC MIN-OBS)
"
  gram-classify-pair-wise COS-CUT FRAC MIN-OBS - Merge words into
  word-classes.

  Very slow, exhaustive O(N^2) algorithm. Suggest using instead
  `gram-classify-agglo`, `gram-classify-diag-blocks` or
  `gram-classify-greedy` for better performance.

  COS-CUT is the minimum cosine between vectors before a merge is
  considered.  Current recomendation is 0.65.

  FRAC is the fraction of the non-overlapping disjuncts that are merged
  into the class. Current recommendation is 0.3.

  MIN-OBS is the smallest number of observations of the word that
  is acceptable; words with fewer observations will be ignored.
"
	(define ZIPF 4)
	(gram-classify classify-pair-wise (make-fuzz COS-CUT FRAC ZIPF MIN-OBS))
)

(define-public (gram-classify-agglo MIN-OBS)
"
  gram-classify-agglo - Merge words into word-classes.

  Conservative O(N^2) algorithm.  Faster than `gram-classify-pair-wise`
  but still slow-ish.  Suggest using instead `gram-classify-diag-blocks`
  or `gram-classify-greedy` for better performance.
"
	(define ZIPF 4)
	(gram-classify agglo-over-words (make-fuzz 0.65 0.3 ZIPF MIN-OBS))
)

(define-public (gram-classify-diag-blocks MIN-OBS)
"
  gram-classify-diag-blocks - Merge words into word-classes.

  Uses a diagonal-block merge strategy. Reasonably fast, better than
  O(N^2) performance, but may miss optimal clusters. Faster than
  `gram-classify-pair-wise` and `gram-classify-agglo`. However, the
  `gram-classify-greedy` variant is both faster and more accurate.
"
	(define ZIPF 4)
	(gram-classify diag-over-words (make-fuzz 0.65 0.3 ZIPF MIN-OBS))
)

(define-public (gram-classify-greedy-fuzz COS-CUT FRAC MIN-OBS)
"
  gram-classify-greedy-fuzz - Merge words into word-classes.

  Uses several tricks to try to get close to O(N log N) performance,
  while retaining high accuracy.  Faster than the exhaustive-search
  `gram-classify-pair-wise` and `gram-classify-agglo` variants. Should
  be faster and more accurate than `gram-classify-diag-blocks`.

  Uses the \"fuzz\" merge algo.

  COS-CUT is the minimum cosine between vectors before a merge is
  considered.  Current recomendation is 0.65.

  FRAC is the fraction of the non-overlapping disjuncts that are merged
  into the class. Current recommendation is 0.3.

  MIN-OBS is the smallest number of observations of the word that
  is acceptable; words with fewer observations will be ignored.
"
	(define ZIPF 4)
	(gram-classify greedy-over-words (make-fuzz COS-CUT FRAC ZIPF MIN-OBS))
)

(define-public (gram-classify-greedy-discrim COSINE MIN-OBS)
"
  gram-classify-greedy-discrim - Merge words into word-classes.

  Uses several tricks to try to get close to O(N log N) performance,
  while retaining high accuracy.  Faster than the exhaustive-search
  `gram-classify-pair-wise` and `gram-classify-agglo` variants. Should
  be faster and more accurate than `gram-classify-diag-blocks`.

  Uses the \"discrim\" merge algo: cosine=0.50, frac=variable

  COSINE should be the minimum cosine angle acceptable to perform
  a merge on. Currently, 0.5 is recommended.

  MIN-OBS is the smallest number of observations of the word that
  is acceptable; words with fewer observations will be ignored.
"
	(define ZIPF 4)
	(gram-classify greedy-over-words (make-discrim COSINE ZIPF MIN-OBS))
)

; ---------------------------------------------------------------
; Example usage
;
; (sql-open "postgres:///en_mst_sections?user=linas&password=asdf")
; (gram-classify-greedy-fuzz 20)
;
; After interruption, its worth recomputing the support marginals.
; This can be done by saying:
;
; (define pca (make-pseudo-cset-api))
; (pca 'fetch-pairs)
; (define psu (add-support-compute pca))
; (psu 'right-marginals)
; (define sto (make-store pca))
; (sto 'store-right-marginals)
;
; The left marginals are not needed, and besides, they take 10x longer
; to compute and store.
