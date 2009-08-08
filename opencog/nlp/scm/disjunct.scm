scm
; 
; disjunct.scm
;
; Build lists of link-grammar disjuncts; update the SQL
; database counts with the results.
;
; This is a part of an experiment in different coding methods.
; A similar set of function is (more completely) implemented in
; disjunct-list.scm, which explores a "list style" programming
; paradigm. This file explores two other paradigms: a 
; callback-style program, and a wiring-style implementation.
;
; XXX This file is more or less dead code, and is not beig currently
; used. XXX
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; =============================================================
; Callback-style disjuncts
;
(define (prt-stuff h) (display h) #f)

; callback-style disjunct stuff.
(define (cb-get-sentence-disjuncts sent-node)

	(define (dj-per-word word)
		(display word)
		; (display (cog-incoming-set word))
	
		#f
	)
	
	; loop over all the words in the sentence
	; XXX this is wrong, this is not how words are indicated
	; (cog-map-chase-link 'SentenceLink 'WordInstanceNode dj-per-word sent-node)
	
	#f
)

; Do it callback-style
; (cog-map-type cb-get-sentence-disjuncts 'SentenceNode)

; ===========================
; wire-style

(define (wire-it)

	; Create a wire to transport a stream of sentences
	(define sentences (make-wire))

	; More wires to transport various bits and pieces.
	(define word-instances (make-wire))
	(define word-nodes (make-wire))
	(define word-instance-pairs (make-wire))
	(define lg-connectors (make-wire))

	; Put the sentences on the wire
	(cgw-source-atoms sentences 'SentenceNode)

	; Get the incoming links.
	; XXX this iis wrong, this is not how words are currently handled.
	; (cgw-follow-link sentences word-instances 'SentenceLink 'WordInstanceNode)
	
	; Get the word-nodes associated with the word-instances.
	; (cgw-follow-link word-instances word-nodes 'ReferenceLink 'WordNode)
	
	(cgw-filter-incoming word-instances word-instance-pairs 'ListLink)

	(cgw-follow-link word-instance-pairs lg-connectors
		'EvaluationLink 'LinkGrammarRelationshipNode)

	; print things out
	(wire-probe "conns" lg-connectors)
)

;===========================

