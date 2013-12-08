;
; link-pipeline.scm
; 
; Link-grammar processing pipeline. Currently, counts word pairs.
;
; Copyright (c) 2103 Linas Vepstas <linasvepstas@gmail.com>
;
; Look for new sentences, count the links in them.


; (get-new-parsed-sentences) returns the sentences
; (release-new-parsed-sents) gets rid of the attachment.
; delete-hypergraph
; cog-atom-incr

; Plan of attack:
; get parses, get links between paris of words, increment counts, store.
;

(define (prt x) (begin (display x) #f))

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
					(proc (cog-get-pred word-inst 'LinkGrammarRelationshipNode))
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

(prt (make-lg-rel x))

(map-lg-links prt
	(get-new-parsed-sentences)
)
