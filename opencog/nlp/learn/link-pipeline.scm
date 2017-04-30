;
; link-pipeline.scm
;
; Link-grammar relation-counting pipeline.  Currently counts only
; word-pairs (links) and not disjuncts. XXX FIXME -- should also
; count disjuncts.
;
; Copyright (c) 2013, 2017 Linas Vepstas <linasvepstas@gmail.com>
;
; This code is part of a language-learning effort.  The goal is to
; observe a lot of text, and then deduce a grammar from it, using
; entropy and other basic probability methods.
;
; Main entry point: (observe-text plain-text)
; Call this entry point with exactly one sentance as a plain text
; string. It will be parsed by RelEx, and the resulting link-grammar
; link usage counts will be updated in the atomspace. The counts are
; flushed to the SQL database so that they're not forgotten.
;
; RelEx is used for only one reason: it prints out the required
; atomese format. The rule-engine in RelEx is NOT used!  This could
; be redesigned and XXX FIXME, it should be.
;
; This tracks multiple, independent counts:
; *) how many sentences have been observed.
; *) how many parses were observed.
; *) how many words have been observed (counting once-per-word)
; *) how many relationship triples have been observed.
;
; Sentences are counted by updating the count on `(SentenceNode "ANY")`.
; Parses are counted by updating the count on `(ParseNode "ANY")`.
; Words are counted by updating the count on the `WordNode` for that
; word. It is counted with multiplicity: once for each time it occurs
; in a parse.  That is, if a word appears twice in a parse, it is counted
; twice.  Counts for relations are stored in the TV for the EvaluationLink.
;

(use-modules (opencog) (opencog nlp) (opencog persist))

; ---------------------------------------------------------------------

(define (count-one-atom ATM)
"
  count-one-atom ATM -- increment the count by one on ATM, and
  update the SQL database to hold that count.

  This will also automatically fetch the previous count from
  the SQL database, so that counting will work correctly, when
  picking up from a previous point.

  Warning: this is NOT SAFE for distributed processing! That is
  because this does NOT grab the count from the database every time,
  so if some other process updates the database, this will miss that
  update.
"
	(define (incr-one atom)
		; If the atom doesn't yet have a count TV attached to it,
		; then its probably a freshly created atom. Go fetch it
		; from SQL. Otherwise, assume that what we've got here,
		; in the atomspace, is the current copy.  This works if
		; there is only one process updating the counts.
		(if (not (cog-ctv? (cog-tv atom)))
			(fetch-atom atom)) ; get from SQL
		(cog-inc-count! atom 1) ; increment
	)
	(begin
		(incr-one ATM) ; increment the count on ATM
		(store-atom ATM)) ; save to SQL
)

; ---------------------------------------------------------------------
; update-word-counts -- update counts for sentences, parses and words,
; for the given list of sentences.
;
; As explained above, the counts on `(SentenceNode "ANY")` and
; `(ParseNode "ANY")` and on `(WordNode "foobar")` are updated.
;
(define (update-word-counts single-sent)
	(define any-sent (SentenceNode "ANY"))
	(define any-parse (ParseNode "ANY"))

	; Due to a RelEx bug in parenthesis handling, the word-inst-get-word
	; function can throw an exception. See detailed explanation furher
	; down. Catch the excpetion, avoid counting if its thrown.
	(define (try-count-one-word word-inst)
		(catch 'wrong-type-arg
			(lambda () (count-one-atom (word-inst-get-word word-inst)))
			(lambda (key . args) #f)))

	(count-one-atom any-sent)
	(map-parses
		(lambda (parse)
			(count-one-atom any-parse)
			(map-word-instances
				try-count-one-word
				parse))
		(list single-sent))
)

; ---------------------------------------------------------------------
; map-lg-links -- loop over all link-grammar links in a list of sentences.
;
; Each link-grammar link is of the general form:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "FOO"
;      ListLink
;         WordInstanceNode "word@uuid"
;         WordInstanceNode "bird@uuid"
;
; The PROC is a function to be invoked on each of these.
;
(define (map-lg-links PROC sent-list)
	(map-parses
		(lambda (parse)
			(map PROC (parse-get-links parse)))
		sent-list)
)

; ---------------------------------------------------------------------
; make-lg-rel -- create a word-relation from a word-instance relation
;
; Get the word relation corresponding to a word-instance relation.
; This simply strips off the unique word-ids from each word. For
; example, given this as input:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "FOO"
;      ListLink
;         WordInstanceNode "word@uuid"
;         WordInstanceNode "bird@uuid"
;
; this creates and returns this:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "FOO" -- gar
;      ListLink                          -- gdr
;         WordNode "word"                -- gadr
;         WordNode "bird"                -- gddr
;
(define (make-lg-rel lg-rel-inst)
	(let (
			(rel-node (gar lg-rel-inst))
			(w-left  (word-inst-get-word (gadr lg-rel-inst)))
			(w-right (word-inst-get-word (gddr lg-rel-inst)))
		)
		(EvaluationLink rel-node (ListLink w-left w-right))
	)
)

; ---------------------------------------------------------------------
; make-word-cset -- create a word-cset from a word-instance cset
;
; A cset is a link-grammar connector set. This takes, as input
; a cset that is attached to a word instance, and creates the
; corresponding cset attached to a word. Basically, it just strips
; off the UUID from the word-instance.
;
; For example, given this input:
;
;   LgWordCset
;      WordInstanceNode "foobar@1233456"
;      LgAnd ...
;
; this creates and returns this:
;
;   LgWordCset
;      WordNode "foobar"  -- gar
;      LgAnd ...          -- gdr
;
(define (make-word-cset CSET-INST)
	(LgWordCset
		(word-inst-get-word (gar CSET-INST))
		(gdr CSET-INST))
)

; ---------------------------------------------------------------------
; update-link-counts -- Increment word and link counts
;
; This routine updates word counts and link counts in the database.
; Word and link counts are needed to compute mutual information (mutual
; entropy), which is required for maximum-entropy-style learning.  The
; algo implemented here is trite: fetch words and relations from SQL;
; increment the attached CountTruthValue; save back to SQL.

(define (update-link-counts single-sent)

	; Due to a RelEx bug, `make-lg-rel` can throw an exception. Specifically,
	; if the word in a sentences is a parentheis, then the ReferenceLink
	; between the spcific paren, and the general par does not get created.
	; Viz, there is no `(ReferenceLink (WordInstanceNode "(@4bf5e341-c6b")
	; (WordNode "("))` ... Either that, or some paren-counter somewhere
	; gets confused. Beats me why. We should fix this. In the meanhile,
	; we hack around this here by catching the exceptioon.  What happens
	; is that the call `word-inst-get-word` returns nothing, so the `car`
	; in `make-lg-rel` throws 'wrong-type-arg and everything gets borken.
	(define (try-count-one-link link)
		(catch 'wrong-type-arg
			(lambda () (count-one-atom (make-lg-rel link)))
			(lambda (key . args) #f)))

	(map-lg-links try-count-one-link (list single-sent))
)

; ---------------------------------------------------------------------
;
; Stupid monitoring utility that can be used to monitor how processing
; is going so far. It counts how many sentences have been processed so
; far. If called with a null argument, it increments the count; else it
; just prints the count.
(define-public monitor-rate
	(let ((mtx (make-mutex))
			(cnt 0)
			(start-time (- (current-time) 0.000001)))
		(lambda (msg)
			(if (null? msg)
				(begin
					(lock-mutex mtx)
					(set! cnt (+ cnt 1))
					(unlock-mutex mtx))
				(format #t "~A cnt=~A rate=~A\n" msg cnt
					(/ cnt (- (current-time) start-time))))
		)))

; ---------------------------------------------------------------------
(define-public (observe-text plain-text)
"
 observe-text -- update word and word-pair counts by observing raw text.

 This is the first part of the learning algo: simply count the words
 and word-pairs oberved in incoming text. This takes in raw text, gets
 it parsed, and then updates the counts for the observed words and word
 pairs.
"
	; Loop -- process any that we find. This will typically race
	; against other threads, but I think that's OK.
	(define (process-sents)
		(let ((sent (get-one-new-sentence)))
			(if (null? sent) '()
				(begin
					(update-word-counts sent)
					(update-link-counts sent)
					(delete-sentence sent)
					(monitor-rate '())
					(process-sents)))))

	(relex-parse plain-text) ;; send plain-text to server
	(process-sents)
	(gc) ;; need agressive gc to keep RAM under control.
)

; ---------------------------------------------------------------------
;
; Some generic hand-testing code for this stuff:
;
; (define (prt x) (display x))
;
; (relex-parse "this is")
; (get-new-parsed-sentences)
;
; (map-lg-links prt (get-new-parsed-sentences))
;
; (map-lg-links (lambda (x) (prt (make-lg-rel x)))
;    (get-new-parsed-sentences))
;
; (map-lg-links (lambda (x) (prt (gddr (make-lg-rel x))))
;    (get-new-parsed-sentences))
;
; (map-lg-links (lambda (x) (cog-inc-count! (make-lg-rel x) 1))
;    (get-new-parsed-sentences))
;
; (observe-text "abcccccccccc  defffffffffffffffff")
