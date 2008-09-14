;
; parse-rank.scm
;
; Provide a score for a link-grammar parse, based on how well it matches
; a minimum spanning tree formed from word-pair mutual information.
;
; Requires access to a database containing previously computed word-pair
; mutual information values.
; 
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; login information and database
(define db-login "linas:asdf:lexat:tcp:localhost:5432")

; (use-modules (dbi dbi))
; (define db-connection (dbi-open "postgresql" db-login))

; Right now, every parse of a sentence is anchored to a ConceptNode
(define ParseAnchor 'ConceptNode)

; Every word of a parse is also a concept node.
(define WordAnchor 'ConceptNode)

(define (prt-stuff h) (display h) #t)
(define (prt-inc h) (display (cog-incoming-set h)) #t)

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
(define (map-parses proc sent) 
	(cog-map-chase-link 'ParseLink ParseAnchor
		" ========= end of sentence ============ \n" ""
		proc sent
	)
)

; map-word-instances proc parse
; Call proceedure 'proc' on each word-instance of 'parse'
;
; Expected input is a ParseAnchor, which serves as an anchor
; to all of the word instances in a parse. The ParseAnchor is
; connnected via a ParseInstanceLink to the individual words.
; 
(define (map-word-instances proc parse) 
	(cog-map-chase-link 'ParseInstanceLink WordAnchor
		" --------- end of parse ------------ \n" ""
		proc parse
	)
)

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

; =============================================================

; Loop over all atoms of type SentenceNode, processing 
; each corresponding parse.
(cog-map-type 
	(lambda (x) (map-parses 
		(lambda (y) (map-word-instances 
			(lambda (z) (map-word-node prt-stuff z))
			y)
		)
	 	x)
	)
	'SentenceNode
)


