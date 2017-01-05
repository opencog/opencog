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

(define (delete-sentence-junk)
"
  delete-sentences       Delete all atoms that occur in sentences

  Private copy of the (delete-sentences) function in the base nlp
  module, for private use by the langaue-learning pipeline. Avoids
  conflict with more mainstream users.

  Delete atoms that belong to particular sentence instances; we don't
  want to clog up the atomspace with old junk we don't want any more.

  Only the atoms in the atomspace are removed; if any are in the
  backingstore (database), they are untouched.
"
	(let ((n 0))
	; (define (delit atom) (set! n (+ n 1)) #f)
	; (define (delit atom) (cog-extract-recursive atom) #f)
	(define (delit atom) (cog-extract-recursive atom) (set! n (+ n 1)) #f)

	; (define (delone atom) (cog-extract atom) #f)
	; (define (delone atom) (cog-extract atom) (set! n (+ n 1)) #f)
	(define (delone atom) (extract-hypergraph atom) (set! n (+ n 1)) #f)

	; Can't delete InheritanceLink, its used to mark wsd completed...
	; (cog-map-type delone 'InheritanceLink)
	; Can't delete EvaluationLink, these are used elsewhere.
	; (cog-map-type delone 'EvaluationLink)
	; Can't delete ListLink, these are used in EvaluationLinks.
	; (cog-map-type delone 'ListLink)

	; These two are used by other subsystems. Currently, it is safe to
	; clobber them here.
	(cog-map-type delone 'PartOfSpeechLink)
	(cog-map-type delone 'LemmaLink)

	(cog-map-type delone 'ParseLink)
	(cog-map-type delone 'ReferenceLink)
	(cog-map-type delone 'LgLinkInstanceLink)

	(cog-map-type delone 'CosenseLink)

	; Its sort of a shame to delete the SimilarityLinks, but these
	; do make the server bloat after a while.
	(cog-map-type delone 'SimilarityLink)

	; We do delete all the test-processing nodes, recursively.
	; This should make any links that contain these to go "poof",
	; and so the above link deletions should not really be needed
	; (except for the similarity links).
	(cog-map-type delit 'DocumentNode)
	(cog-map-type delit 'SentenceNode)
	(cog-map-type delit 'ParseNode)
	(cog-map-type delit 'WordInstanceNode)
	(cog-map-type delit 'LgLinkInstanceNode)

	; Pointless to delete these, since there should only be
	; a few hundred of these, total.
	; (cog-map-type delit 'LinkGrammarRelationshipNode)
	; (cog-map-type delit 'DefinedLinguisticConceptNode)
	; (cog-map-type delit 'DefinedLinguisticRelationshipNode)

	; Lots of junk NumberNodes get created. Remove these last, after
	; the various owners have been deleted.
	(cog-map-type delone 'NumberNode)

(system (string-join (list "echo deleted: " (number->string n) )))
	)
)
; ---------------------------------------------------------------------
(define-public (observe-text plain-text)
"
 observe-text -- update word and word-pair counts by observing raw text.

 This is the first part of the learning algo: simply count the words
 and word-pairs oberved in incoming text. This takes in raw text, gets
 it parsed, and then updates the counts for the observed words and word
 pairs.
"
	(begin
		(relex-parse plain-text) ;; send plain-text to server
		(update-link-counts (get-new-parsed-sentences))
		(release-new-parsed-sents)
		(delete-sentence-junk)
	)
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
; (map-lg-links (lambda (x) (cog-atom-incr (make-lg-rel x) 1))
; 	(get-new-parsed-sentences)
; )
; 
; (observe-text "abcccccccccc  defffffffffffffffff")
