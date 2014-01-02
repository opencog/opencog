;
; link-pipeline.scm
; 
; Link-grammar processing pipeline. Currently, just counts word pairs.
;
; Copyright (c) 2013 Linas Vepstas <linasvepstas@gmail.com>
;
; This code is part of a language-learning effort.  The goal is to
; observe a lot of text, and then perform clustering to min-max the
; entropy. Right now, nly the observe part is supported.
;
; Main entry point: (observe-text plain-text)
; Call this entry point with exactly one sentance as plain text.
; It will be parsed by RelEx, and the resulting link-grammar link
; usage counts will be updated in the atomspace. The counts are
; flushed to the SQL database so that they're not forgotten.
;
; As of 10 December 2013, this seems to work fine. Deploying ...
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
	(map-parses
		(lambda (parse)
			(map-word-instances
				(lambda (word-inst)
					(begin
						(map proc (cog-get-pred word-inst 'LinkGrammarRelationshipNode))
						#f
					)
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
; Word and link counts are needed to compute mutial information (mutual
; entropy), which is required for maximum-entropy-style learning.  The
; algo implemented here is trite: fetch words and relations from SQL;
; increment the attached CountTruthValue; save back to SQL.  Note:
; we are not directly accessing the database; we're just letting the
; atomspace handle that semi-automatically.

(define (update-link-counts sents)
	(define (count-one-link link)
		(let ((rel (make-lg-rel link)))
			(begin
				; Explicit fetch not needed; it already happens implicitly.
				; (Assuming there are no other servers modifying SQL.)
				; (fetch-atom rel) ; get from SQL
				(cog-atom-incr rel 1) ; increment relation
				(cog-atom-incr (gar rel) 1)  ; increment link type
				(cog-atom-incr (gadr rel) 1) ; increment left word
				(cog-atom-incr (gddr rel) 1) ; increment right work.
				(store-atom rel) ; save to SQL
				#f ; need to return #f so that map-lg-links doesn't stop.
			)
		)
	)
	(map-lg-links count-one-link sents)
)

; ---------------------------------------------------------------------
; observe-text -- update word and word-pair counts by observing raw text.
;
; This is the first part of the learning algo: simply count the words
; and word-pairs oberved in incoming text. This takes in raw text, gets
; it parsed, and then updates the counts for the observed words and word
; pairs.
(define (observe-text plain-text)
	(begin
		(relex-parse plain-text)  ;; send plain-text to server
		(update-link-counts (get-new-parsed-sentences))
		(release-new-parsed-sents)
		(delete-sentences)
	)
)

; ---------------------------------------------------------------------
;
; Some generic hand-testing code for this stuff:
;
; (define (prt x) (begin (display x) #f))
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
; (map-lg-links (lambda (x) (cog-atom-incr (make-lg-rel x) 1))
; 	(get-new-parsed-sentences)
; )
; 
; (observe-text "abcccccccccc  defffffffffffffffff")
