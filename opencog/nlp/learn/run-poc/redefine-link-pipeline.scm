; update-clique-pair-counts -- count occurrences of random word-pairs.
;
; This generates what are termed "clique pairs" throughout: these are
; all possible word-pair combinations, given a sequence of words.
; No parsing is involved; this code simply generates one word-pair
; for each and every edge in the clique of the sequence of the words.
;
; This code is problematic for multiple reasons:
; 1) The kinds of pairs it generates occur with different frequencies
;    than they would in a random planar tree parse.  In particular,
;    it generates more pairs between distant words than the planar tree
;    would. This could be ameliorated by simply not generating pairs
;    for words that are more than 6 lengths apart. Or, alternately,
;    only the statistics for closer pairs closer together than 6 could
;    be used.  Anyway, this is probably not a big deal, by itself.
;
; 2) This generates pairs tagged with the distance between the pairs.
;    (See below for the format).  This is might be interesting to
;    look at for academic reasons, but it currently puts a huge
;    impact on the size of the atomspace, and the size of the
;    database, impacting performance in a sharply negative way.
;    That's because, for every possible word-pair, chances are that
;    it will appear, sooner or later, with with every possible distance
;    from 1 to about 30. Each distance requires it's own atom to keep
;    count: thus requiring maybe 30x more atoms for word-pairs!  Ouch!
;    This is huge!
;
;    Limiting pair-counts to distances of 6 or less still blows up
;    the database size by 6x... which is still a lot.
;
;    We might be able to cut down on this by using different values
;    (on the same pair-atom) to count the different lengths, but the
;    hit is still huge.
;
; 3) On a per-sentence basis, when clique-counting is turned on, the
;    number of database updates increases by 3x-4x atom value updates.
;    If your database is on spinning disks, not SSD, this means that
;    database updates will be limited by the disk I/O subsystem, and
;    this additional traffic can slow down statistics gathering by...
;    3x or 4x.
;
; Clique-counting can be used by passing "clique" or "clique-dist"
; as second argument when calling observe-text-mode.
;
; The structures that get created and incremented are of the form
;
;     EvaluationLink
;         PredicateNode "*-Sentence Word Pair-*"
;         ListLink
;             WordNode "lefty"  -- or whatever words these are.
;             WordNode "righty"
;
;     ExecutionLink
;         SchemaNode "*-Pair Distance-*"
;         ListLink
;             WordNode "lefty"
;             WordNode "righty"
;         NumberNode 3
;
; Here, the NumberNode encodes the distance between the words. It is always
; at least one -- i.e. it is the difference between their ordinals.
;
; Parameters:
; DIST-MODE -- booleant #t or #f: enable or disable multiplying each
; 			   pair-count by a distance weight given by
;			   (quotient MAX-LEN dist), where dist is the separation between
;			   the words in the pair.
; MAX-LEN -- integer: don't count a pair, if the words are farther apart
;            than this.
; RECORD-LEN -- boolean #t of #f: enable or disable recording of lengths.
;            If enabled, see warning about the quantity of data, above.
;
(define (update-pair-counts-once PARSE DIST-MODE MAX-LEN RECORD-LEN)

	; Function to calculate how many times to count a word-pair
	(define calc-times
		(if DIST-MODE
			(lambda (d) (quotient MAX-LEN d))
			(lambda (d) 1)))

	; Get the scheme-number of the word-sequence number
	(define (get-no seq-lnk)
		(string->number (cog-name (gdr seq-lnk))))

	; Create and count a word-pair, and the distance.
	(define (count-one-pair left-seq right-seq)
		(define dist (- (get-no right-seq) (get-no left-seq)))

		; Only count if the distance is less than the cap.
		(if (<= dist MAX-LEN)
			(let ((pare (ListLink (gar left-seq) (gar right-seq)))
				(counts (calc-times dist)))
				(count-one-atom-times (EvaluationLink pair-pred pare) counts)
				(if RECORD-LEN
					(count-one-atom
						(ExecutionLink pair-dist pare (NumberNode dist)))))))

	; Create pairs from `first`, and each word in the list in `rest`,
	; and increment counts on these pairs.
	(define (count-pairs first rest)
		(if (not (null? rest))
			(begin
				(count-one-pair first (car rest))
				(count-pairs first (cdr rest)))))

	; Iterate over all of the words in the word-list, making pairs.
	(define (make-pairs word-list)
		(if (not (null? word-list))
			(begin
				(count-pairs (car word-list) (cdr word-list))
				(make-pairs (cdr word-list)))))

	; If this function throws, then it will be here, so all counting
	; will be skipped, if any one word fails.
	(define word-seq (make-word-sequence PARSE))

	; What the heck. Go ahead and count these, too.
	(for-each count-one-atom word-seq)

	; Count the pairs, too.
	(make-pairs word-seq)
)

; wrapper for backwards compatibility
(define (update-clique-pair-counts SENT MAX-LEN RECORD-LEN)
	(update-clique-pair-counts-mode SENT #f MAX-LEN RECORD-LEN))

; See above for explanation.
(define (update-clique-pair-counts-mode SENT DIST-MODE MAX-LEN RECORD-LEN)
	; In most cases, all parses return the same words in the same order.
	; Thus, counting only requires us to look at only one parse.
	(update-pair-counts-once
		(car (sentence-get-parses SENT))
		DIST-MODE MAX-LEN RECORD-LEN)
)

; ---------------------------------------------------------------------
(define-public (observe-text-mode plain-text observe-mode count-reach)
"
 observe-text-mode -- update word and word-pair counts by observing raw text.

 There are currently three observing modes, set by observe-mode, all taking
 an integer parameter:
 - any: counts pairs of words linked by the LG parser in 'any' language.
        In this case, 'count-reach' specifies how many linkages from LG-parser
        to use.
 - clique: itearates over each word in the sentence and pairs it with
           every word located within distance 'count-reach' to its right.
           Distance is defined as the difference between words positions
           in the sentence, so neighboring words have distance of 1.
 - clique-dist: same word-pairs as 'clique', but each word-pair is counted
                a number of times determined by the distance between words
                in the pair as:
                (quotient count-reach distance)

 This is the first part of the learning algo: simply count the words
 and word-pairs observed in incoming text. This takes in raw text, gets
 it parsed, and then updates the counts for the observed words and word
 pairs.
"
	; try-catch wrapper around the counters. Due to a buggy RelEx
	; (see documentation for `word-inst-get-word`), the function
	; `update-clique-pair-counts` might throw.  If it does throw,
	; then avoid doing any counting at all for this sentence.
	;
	; Note: update-clique-pair-counts commented out. If you want this,
	; then uncomment it, and adjust the length.
	; Note: update-disjunct-counts commented out. It generates some
	; data, but none of it will be interesting to most people.
	(define (update-counts sent)
		(catch 'wrong-type-arg
			(lambda () (begin
				; 6 == max distance between words to count.
				; See docs above for explanation.
				; (update-clique-pair-counts sent 6 #f)
				(update-word-counts sent)
				(update-lg-link-counts sent)
				; If you uncomment this, be sure to also uncomment
				; LgParseLink below, because LgParseMinimal is not enough.
				; (update-disjunct-counts sent)
			))
			(lambda (key . args) #f)))

	; Count the atoms in the sentence, according to the counting method
	; passed as argument, then delete the sentence.
	(define (process-sent SENT cnt-mode win-size)
		(update-word-counts SENT)
		(cond
		 	((equal? cnt-mode "any") (update-lg-link-counts SENT))
			((equal? cnt-mode "clique") (update-clique-pair-counts-mode SENT #f win-size #f))
			((equal? cnt-mode "clique-dist") (update-clique-pair-counts-mode SENT #t win-size #f)))
		(delete-sentence SENT)
		(monitor-parse-rate '()))

	; -------------------------------------------------------
	; Manually run the garbage collector, every now and then.
	; This helps keep RAM usage down, which is handy on small-RAM
	; machines. However, it does cost CPU time, in exchange.
	; Adjust `how-often` up or down to suit your whims.
	(define sometimes-gc
		(let ((cnt 0)
				(how-often 10)) ; every 10 times.
			(lambda ()
				(set! cnt (+ cnt 1))
				(if (eqv? 0 (modulo cnt how-often)) (gc)))))

	; Perform GC whenever it gets larger than a fixed limit.
	; Less than one GB should be enough, but the huge strings
	; from relex seem to cause bad memory fragmentation.
	(define maybe-gc
		(let ((cnt 0)
				(max-size (* 2750 1000 1000)))  ; 750 MB
			(lambda ()
				(if (< max-size (- (assoc-ref (gc-stats) 'heap-size)
							(assoc-ref (gc-stats) 'heap-free-size)))
					(begin
						(gc)
						(set! cnt (+ cnt 1))
						;(report-avg-gc-cpu-time)
					)))))

	; Caution: RelEx-bassed parsing is deprecated, for three reasons:
	; 1) Its slower, because it has to go to the relex server;
	; 2) The RelEx server returns long strings containing scheme,
	;    (often up to 8MBytes long) which cause HUGE GC issues for
	;    guile - blowing up RAM usage.
	; 3) The current guile-2.2 compiler has a bug compiling code
	;    obtained from strings, and intermittently crashes. (A bug
	;    report has been submitted, but is missing code to reproduce
	;    it, and so is not getting fixed...)
	;
	; Use the RelEx server to parse the text via Link Grammar.
	; Return a SentenceNode. Attention: when run in parallel,
	; the returned SentenceNode is not necessarily that of the
	; the one that was submitted for parsing! It might be just
	; some other sentence that is sitting there, ready to go.
	(define (relex-process TXT)
		(define (do-all-sents)
			(let ((sent (get-one-new-sentence)))
				(if (not (null? sent))
					(begin (process-sent sent) (do-all-sents)))))

		(relex-parse TXT)
		(do-all-sents)
		(maybe-gc) ;; need agressive gc to keep RAM under control.
	)

	; Process the text locally (in RAM), with the LG API link or clique-count.
	(define (local-process TXT obs-mode cnt-reach)
		; try-catch wrapper for duplicated text. Here's the problem:
		; If this routine is called in rapid succession with the same
		; block of text, then only one PhraseNode and LgParseLink will
		; be created for both calls.  The extract at the end will remove
		; this, even while these atoms are being accessed by the second
		; call.  Thus, `lgn` might throw because `phr` doesn't exist, or
		; `cog-execute!` might throw because lgn does't exist. Either of
		; the cog-extracts might also throw. Hide this messiness.
		(catch #t
			(lambda ()
				(let* ((phr (Phrase TXT))
						; needs at least one linkage for tokenization
						(num-parses (if (equal? obs-mode "any") cnt-reach 1))
						(lgn (LgParseMinimal phr (LgDict "any") (Number num-parses)))
						(sent (cog-execute! lgn))
					)
					(process-sent sent obs-mode cnt-reach)
					; Remove crud so it doesn't build up.
					(cog-extract lgn)
					(cog-extract phr)
				))
			(lambda (key . args) #f))
	)

	;; Send plain-text to the relex server
	; (relex-process plain-text)

	; Handle the plain-text locally
	(local-process plain-text observe-mode count-reach)
)
