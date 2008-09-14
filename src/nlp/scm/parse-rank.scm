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
(define (prt-words h) 
	(define (get-word w)
		(cog-filter WordAnchor prt-stuff (cog-outgoing-set w))
		#f
	)
	(cog-filter 'ParseInstanceLink get-word (cog-incoming-set h))
	(display " --------- end of parse ------------ \n")
	#f
)

; Expected input is a SentenceNode, which serves as an anchor to all
; of the parses of a sentence. It is connected via ParseLink's to 
; each of the individual parses of the sentence. So, look for 
; ParseAnchor's, which anchor the parses.
(define (process-parses h) 
	(define (get-parse p)
		(cog-filter ParseAnchor prt-words (cog-outgoing-set p))
		#f
	)
	(cog-filter 'ParseLink get-parse (cog-incoming-set h))
	(display " ========= end of sentence ============ \n")
	#f
)

; Loop over all atoms of type SentenceNode, processing 
; each corresponding parse.
(cog-map-type process-parses 'SentenceNode)


