;
; nlp-utils.scm
;
; Assorted NLP utilities.
; 
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; =============================================================

; Right now, every parse of a sentence is anchored to a ParseNode
(define ParseAnchor 'ParseNode)

; Every word of a parse is a WordInstance node.
(define WordAnchor 'WordInstanceNode)

; =============================================================
; Assorted sentence, parse, word, word-sense wrangling utilities below.
;
; map-parses proc sent
; Call proceedure 'proc' on every parse of the sentence 'sent' 
; 
; Expected input is a SentenceNode, which serves as an anchor to all
; of the parses of a sentence. It is connected via ParseLink's to 
; each of the individual parses of the sentence. So, look for 
; ParseAnchor's, which anchor the parses.
;
; Just as an or-map, the recursion will stop if proc returns something
; other than #f. This routine returns the last value that stopped the
; recusrsion.
;
(define (map-parses proc sent) 
   (if (not (eq? (cog-type sent) 'SentenceNode))
      (throw 'wrong-atom-type 'map-parses
          "Error: expecting SentenceNode" sent)
   )
	(cog-map-chase-link 'ParseLink ParseAnchor
		" ========= sentence ============ \n" ""
		proc sent
	)
)

; ---------------------------------------------------------------------
; map-word-instances proc parse
; Call proceedure 'proc' on each word-instance of 'parse'
;
; Expected input is a ParseAnchor, which serves as an anchor
; to all of the word instances in a parse. The ParseAnchor is
; connnected via a ParseInstanceLink to the individual words.
; 
(define (map-word-instances proc parse) 
	(cog-map-chase-link 'ParseInstanceLink WordAnchor
		" --------- parse ------------ \n" ""
		proc parse
	)
)

; ---------------------------------------------------------------------
; map-word-node proc word-inst
; Call proceedure 'proc' on the word-node associated to 'word-inst'
;
; Expected input is a WordAnchor, which serves as an anchor
; to a word instance. The WordAnchor is connnected via a ReferenceLink
; to the actual word node.
;
(define (map-word-node proc word-inst) 
	(cog-map-chase-link 'ReferenceLink 'WordNode 
		"" ""  ; " --------- word-found ------------ \n"
		proc word-inst
	)
)

; ---------------------------------------------------------------------
; Return the WordNode associated with 'word-inst'
;
(define (word-inst-get-word word-inst)
	(cog-chase-link 'ReferenceLink 'WordNode word-inst)
)

; ---------------------------------------------------------------------
; Return the word string associated with the word-instance
(define (word-inst-get-word-str word-inst)
	(cog-name (car (word-inst-get-word word-inst)))
)

; ---------------------------------------------------------------------
; Given a word instance, return the lemma for of the word.
(define (word-inst-get-lemma word-inst)
	(car (cog-chase-link 'LemmaLink 'WordNode word-inst))
)

; ---------------------------------------------------------------------
; Given a word instance, return a list of attributes for the word.
(define (word-inst-get-attr word-inst)
	(cog-chase-link 'InheritanceLink 'DefinedLinguisticConceptNode 
		word-inst
	)
)

; ---------------------------------------------------------------------
; Given a word instance, return the inflection string for it.
; The "inflection string" is the part that follows the period in
; the link-grammar dictionary. Thus, for the dictionary entry
; "red.a" the ".a" is the inflection.  Note that not all link-grammar
; dictionary entries will have an inflection.
;
(define (word-inst-get-inflection-str word-inst)
	; all inflections start with a period as the first character.
	(define (inflection? tag)
		(if (char=? #\. (string-ref (cog-name tag) 0))
			#t #f
		)
	)
	(let ((infl (filter! inflection? (word-inst-get-attr word-inst))))
		(if (null? infl)
			""
			(cog-name (car infl))
		)
	)
)

; ---------------------------------------------------------------------
; Given a word instance, return the inflected word.
; Here, "inflected words" are link-grammar dictionary entries, for 
; example, "events.n" or "offered.v". Note that not all link-grammar
; dictionary entries will have an inflection.
;
(define (word-inst-get-inflected-word-str word-inst)
	(string-append
		(word-inst-get-word-str word-inst)
		(word-inst-get-inflection-str word-inst)
	)
)

; ---------------------------------------------------------------------
; Given a word instance, return a list of the word-senses associated with the word.
(define (word-inst-get-senses word-inst)
	(cog-chase-link 'InheritanceLink 'WordSenseNode word-inst)
)

; ---------------------------------------------------------------------
; Given a word-instance and a word-sense, return the numerical 
; word-sense ranking score. The rank is stored as the "count" part of the
; count truth value of the InheritanceLink connecting the two: i.e. in
; the structure
;
;     InheritanceLink (ctv 1.0 0.0 -0.006025)
;        WordInstanceNode "day@171506d8f3"
;        WordSenseNode "daylight%1:28:00::"
;
; If there is no such link, return -999 as the score.
;
(define (word-inst-sense-score word-inst word-sense)
	(let ((slink (cog-link 'InheritanceLink word-inst word-sense)))
		(if (null? slink)
			-999.0
			(cdr (assoc 'count (cog-tv->alist (cog-tv slink))))
		)
	)
) 

; ---------------------------------------------------------------------
; Given a document, return a list of sentences in that document
;
(define (document-get-sentences doco)
	(cog-get-reference doco)
)

; ---------------------------------------------------------------------
; Given a sentence, return a list of parses in that sentence
; Basically, chase a ParseLink to a ParseNode
;
(define (sentence-get-parses sent-node)
	(cog-chase-link 'ParseLink 'ParseNode sent-node)
)

; ---------------------------------------------------------------------
; Given a parse, return a list of all words in the parse
;
(define (parse-get-words parse-node)
	(cog-outgoing-set (car (cog-chase-link 'ReferenceLink 'ListLink parse-node)))
)

; ---------------------------------------------------------------------
; Delete atoms that belong to particular sentence instances; we don't
; want to log up the server with grunge text.
;
(define (delete-sentences)
	(let ((n 0))
	; (define (delit atom) (set! n (+ n 1)) #f)
	; (define (delit atom) (cog-delete-recursive atom) #f)
	(define (delit atom) (cog-delete-recursive atom) (set! n (+ n 1)) #f)

	; (define (delone atom) (cog-delete atom) #f)
	(define (delone atom) (cog-delete atom) (set! n (+ n 1)) #f)

	; Can't delete InheritanceLink, its used to mark wsd completed... 
	; (cog-map-type delone 'InheritanceLink)
	; Can't delete EvaluationLink, these are used elsewhere.
	; (cog-map-type delone 'EvaluationLink)
	; Can't delete ListLink, these are used in EvaluationLinks.
	; (cog-map-type delone 'ListLink)
	
	; Part of Speech links are used in the word-sense 
	; database, so cannot delete these.
	; (cog-map-type delone 'PartOfSpeechLink)
	; Can't delete LemmaLink's; used by the triples processing rules
	; (cog-map-type delone 'LemmaLink)

	(cog-map-type delone 'ParseLink)
	(cog-map-type delone 'ReferenceLink)

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

	; Pointless to delete these, since there should only be 
	; a few hundred of these, total.
	; (cog-map-type delit 'LinkGrammarRelationshipNode) 
	; (cog-map-type delit 'DefinedLinguisticConceptNode) 
	; (cog-map-type delit 'DefinedLinguisticRelationshipNode) 

(system (string-join (list "echo deleted: " (number->string n) )))
	)
)

; =============================================================
