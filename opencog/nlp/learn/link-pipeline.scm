;
; link-pipeline.scm
;
; Link-grammar processing pipeline. Currently, just counts word pairs.
;
; Copyright (c) 2013 Linas Vepstas <linasvepstas@gmail.com>
;
; This code is part of a language-learning effort.  The goal is to
; observe a lot of text, and then perform clustering to min-max the
; entropy. Right now, only the observe part is supported.
;
; Main entry point: (observe-text plain-text)
; Call this entry point with exactly one sentance as plain text.
; It will be parsed by RelEx, and the resulting link-grammar link
; usage counts will be updated in the atomspace. The counts are
; flushed to the SQL database so that they're not forgotten.
;
; As of 10 December 2013, this seems to work fine. Deploying ...

(use-modules (opencog) (opencog nlp) (opencog persist))
;
; ---------------------------------------------------------------------
; map-lg-links -- loop over all link-grammar links in sentences.
;
; Each link-grammar link is of the general form:
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordInstanceNode "word@uuid"
;         WordInstanceNode "bird@uuid"
;
; and 'proc' is invoked on each of these.
;
; Note -- as currently written, this double-counts.
(define (map-lg-links proc sent-list)
	; Do each parse in it's own thread.
	; (parallel-map-parses
	(map-parses
		(lambda (parse)
			(map-word-instances
				(lambda (word-inst)
					(map proc (cog-get-pred word-inst 'LinkGrammarRelationshipNode))
				)
				parse
			)
		)
		sent-list
	)
)

; ---------------------------------------------------------------------
; make-lg-rel -- create a word-relation from a word-instance relation
;
; Get the word relation correspoding to a word-instance relation.
; That is, given this:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordInstanceNode "word@uuid"
;         WordInstanceNode "bird@uuid"
;
; create this:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         WordNode "bird"
;
(define (make-lg-rel lg-rel-inst)
	(let (
			(rel-node (gar lg-rel-inst))
			(w-left  (car (word-inst-get-word (gadr lg-rel-inst))))
			(w-right (car (word-inst-get-word (gddr lg-rel-inst))))
		)
		(EvaluationLink rel-node (ListLink w-left w-right))
	)
)

; ---------------------------------------------------------------------
; update-link-counts -- Increment word and link counts
;
; This routine updates word counts and link counts in the database.
; Word and link counts are needed to compute mutual information (mutual
; entropy), which is required for maximum-entropy-style learning.  The
; algo implemented here is trite: fetch words and relations from SQL;
; increment the attached CountTruthValue; save back to SQL.

(define (update-link-counts sents)
	(define (count-one-link link)
		(define (incr-one atom)
			; If the atom doesn't yet have a count TV attached to it,
			; then its probably a freshly created atom. Go fetch it
			; from SQL. Otherwise, assume that what we've got here,
			; in the atomspace, is the current copy.  This works if
			; there is only once cogserver updating the counts.
			(if (not (cog-ctv? (cog-tv atom)))
				(fetch-atom atom)) ; get from SQL
			(cog-inc-count! atom 1) ; increment
		)
		(let ((rel (make-lg-rel link)))
			(begin
				; Evalu -- rel
				;   Pred -- gar
				;   List -- gdr
				;     Left  -- gadr
				;     Right -- gddr
				(incr-one rel) ; increment relation (the evaluation link)
				(incr-one (gar rel))  ; increment link type
				(incr-one (gadr rel)) ; increment left word
				(incr-one (gddr rel)) ; increment right work.
				(store-atom rel) ; save (recursively) to SQL
			)
		)
	)

	; Under weird circumstances, the above can fail. Specifically,
	; sometimes RelEx doesn't generate the needed (ReferenceLink
	; connecting (WordInstanceNode "(@4bf5e341-c6b") to the desired
	; (WordNode "(") ... Either that, some paren-counter somewhere
	; gets confused. Beats me why. We should fix this. In the meanhile,
	; we hack around this here by catching.  What happens is that the
	; call (word-inst-get-word ...) returns nothing, so the car in
	; make-lg-rel throws 'wrong-type-arg and everything gets borken.
	(define (try-count-one-link link)
		(catch 'wrong-type-arg
			(lambda () (count-one-link link))
			(lambda (key . args) #f)
		)
	)
	(map-lg-links try-count-one-link sents)
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
					(update-link-counts (list sent))
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
; 	(get-new-parsed-sentences)
; )
;
; (map-lg-links (lambda (x) (prt (gddr (make-lg-rel x))))
; 	(get-new-parsed-sentences)
; )
;
; (map-lg-links (lambda (x) (cog-inc-count! (make-lg-rel x) 1))
; 	(get-new-parsed-sentences)
; )
;
; (observe-text "abcccccccccc  defffffffffffffffff")
