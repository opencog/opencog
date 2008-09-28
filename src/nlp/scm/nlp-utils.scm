scm
;
; nlp-utils.scm
;
; Assorted NLP utilities.
; 
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; =============================================================

; Right now, every parse of a sentence is anchored to a ConceptNode
(define ParseAnchor 'ConceptNode)

; Every word of a parse is also a concept node.
(define WordAnchor 'ConceptNode)

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
(define (get-word word-inst)
	(cog-chase-link 'ReferenceLink 'WordNode word-inst)
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
; Given a parse, return a list of all the link-grammar linkage relations
; for that parse.
;
(define (parse-get-lg-relations parse-node)
   (cog-get-reference parse-node)
)

; Given a parse, return a list of all words in the parse
(define (parse-get-words parse-node)
	(cog-outgoing-set (car (cog-chase-link 'ReferenceLink 'ListLink parse-node)))
)

; =============================================================
.
exit
