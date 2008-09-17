scm
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

(turn-on-debugging)
(use-modules (dbi dbi))
(define db-connection (dbi-open "postgresql" db-login))

;
; The parse ranker works by adding up the mutual-information scores
; associated with each link-grammar link. The higher the total mutual
; information, the higher the parse is ranked.
;
; Link-grammar structure
; ----------------------
; The code below assumes that opencog contains the link-grammar links 
; describing the parse. The assumed format for these links is as in 
; the following example:
; 
;   (EvaluationLink
;      (LinkGrammarRelationshipNode "EB")
;      (ListLink
;         (ConceptNode "consist@79aac665-c59f-41e2-9ab6-af75b768b5c4")
;         (ConceptNode "mostly@69de3109-9e54-4acc-8aef-66e9da304078")
;      )
;   )
;
; where "mostly@69de3109-9e54-4acc-8aef-66e9da304078" is a typical 
; word-instance node.

; misc debugging crap
(define (prt-stuff h) (display h) #f)
(define (prt-inc h) (display (cog-incoming-set h)) #t)
(define (prt-word h) (display (cog-name h)) (newline) #f)


; Get the mutual info for word-pairs in a sentence.
; Expects an atom of type SentenceLink as input.
; Returns a list of MI-weighted word-pairs for the sentence.
; 
; Consider the set of all pairs of words in a sentence (with the left
; word of the pair appearing to the left in the sentence of the right
; word in the pair).  This list of pairs can be thought to be a list of
; edges for a graph (whose vertices are the words themselves).  The
; graph is a clique.
;
; This routine will look up the mutual-information scores for each
; word-pair from a database, and associate that score with the word-pair.
; If there's no score in the database, the word-pair is omitted from 
; the list. This routine returns that association list.
;
(define (get-mutual-info sentence)

	; Accumulate words into a list; called from a deep nested loop.
	(define word-list '())
	(define (add-to-word-list h)
		(set! word-list (cons (cog-name h) word-list))
		#f
	)

	; Given list l, return the list in reverse order
	(define (reverse-list l)
		(define (rl ol nl)
			(if (eq? ol '())
				nl
				(rl (cdr ol) (cons (car ol) nl))
			)
		)
		(rl l '())
	)

	; Make a list of left-right pairs of all words in 'word-list'
	; That is, the right word of each pair must be to the right of the
	; left word in each pair (in sentential order). So, for example,
	; "Hello world" will produce only one pair: (hello . world) and
	; "Good moring world" produces three pairs: (good . morning)
	; (good . world) (morning . world)
	;
	; Returns a list of pairs.
	;
	; This list of pairs should be thought of as a graph, whose
	; edges are the pairs, and whose vertexes are the individual words.
	(define (make-word-pairs word-list)
		; Make a list of pairs out of 'word' and 'word-list'
		; Return this list of pairs.
		(define (mk-pair word wd-list prs)
			(if (eq? wd-list '())
				prs
				(mk-pair word (cdr wd-list)
					(cons (cons word (car wd-list)) prs)
				)
			)
		)
		; tail-recursive helper function to make the actual list
		(define (mk-prs wlist prl)
			(if (eq? wlist '())
				prl
				(mk-prs (cdr wlist)
					(mk-pair (car wlist) (cdr wlist) prl))
			)
		)
		(mk-prs word-list '())
	)

	; Given a word-pair, look it up in the frequency/mutal-info database,
	; and associate a mutal info score to it. Return the scored pair,
	; else return empty list.  So, example, the pair (pile .of) produces
	; ((pile . of) . 3.73598462236839)
	;
	; Returns the triple (the weighted edge)
	;
	; The score should be understood to be a weight assciated with the
	; graph edge.
	(define (score-edge pr)
		(define qstr
			(string-append
				"SELECT * FROM pairs WHERE left_word='"
				(car pr) "' AND right_word='" (cdr pr) "'"))

		(define row #f)
		; (display qstr) (newline)
		(dbi-query db-connection qstr)
		(set! row (dbi-get_row db-connection))
		(if (eq? row #f)
			'()
			(cons pr (cdr (assoc "mutual_info" row)))
		)
	)

	; Iterate over a list of pairs, and fetch word-pair scores from
	; the SQL database. Returns an association list of scored pairs.
	; The returned list should be understood to be a list of weighted
	; graph edges.
	(define (score-graph pair-list)
		(define (sc-graph pairlist edge-list)
			(if (eq? pairlist '())
				edge-list
				(let ((score (score-edge (car pairlist))))
					(if (eq? score '())
						(sc-graph (cdr pairlist) edge-list)
						(sc-graph (cdr pairlist) (cons score edge-list))
					)
				)
			)
		)
		(sc-graph pair-list '())
	)

	; Iterate over a list graph edges, and return the edge with the
	; highest weight.
	; XXX Umm, actually, this routine was intended for a
	; maximum-spanning-tree algorithm, but we don't actually need that
	; algo to properly rank parse scores. So this is actually some dead
	; code that we're not using just right now ...
	(define (find-highest-score edge-list)
		(define (find-hig elist best)
			(if (eq? elist '())
				best
				(if (< (cdr best) (cdr (car elist)))
					(find-hig (cdr elist) (car elist))
					(find-hig (cdr elist) best)
				)
			)
		)
		(find-hig (cdr edge-list) (car edge-list))
	)

	; Validate our input -- we're expecting a SentenceLink
	(if (not (eq? (cog-type sentence) 'SentenceLink))
		(throw 'wrong-atom-type 'get-mutual-info
			 "Error: expecting SentenceLink" sentence)
	)

	; Create a list of all of the word instances in the parse.
	; (cog-outgoing-set sentence) is a list of all of the word-instances
	; in the sentence. Get those (filter on ConceptNode in order
	; to ignore the SentenceNode), then map these to word nodes, 
	; and then tack them onto our list.
	(cog-filter 'ConceptNode 
		(lambda (x) (map-word-node add-to-word-list x))
		(cog-outgoing-set sentence)
	)

	; (display word-list)
	; (display (make-word-pairs word-list) )
	; (display (score-graph (make-word-pairs (reverse-list word-list))))
	; (display (find-highest-score 
	;	(score-graph (make-word-pairs (reverse-list word-list)))))
	; (newline)
	
	; Return a list of weighted edges for the graph.
	(score-graph (make-word-pairs (reverse-list word-list)))
)

; =============================================================

; Adjust the parse ranking of each parse.
; Accepts an atom of type SentenceNode as input
(define (score-sentence sent-node)

	; Get the list of weighted edges
	(define mi-edge-list '())
	(define (get-mi sent-link)
		(set! mi-edge-list (get-mutual-info sent-link))
		#f
	)

	; Invoke procedure 'proc' on each link-grammar link.
	; Expects as input a ParseAnchor (node anchoring a parse)
	; Returns whatever proc returns, as an or-map.
	(define (map-links proc parse-inst)
   	(cog-map-chase-link 'LinkGrammarLinkageLink 'EvaluationLink
      	""  " --------- linkage-found ------------ \n"
      	proc parse-inst
   	)
	)

	; build a word pair
	; Expects as input a link-grammar link
	(define (make-lg-pair lg-link)
		; strip off EvaluationLink, and then ListLink
		(let* (
				(raw-pair (cog-outgoing-set (cadr (cog-outgoing-set lg-link))))
				(left-raw (car raw-pair))
				(right-raw (cadr raw-pair))
			)
			(display left-raw)
			(display right-raw)
			(newline)
		)
		#f
	)

	(cog-filter 'SentenceLink get-mi (cog-incoming-set sent-node))

	; (display mi-edge-list) (newline)
	; peek
	(display sent-node)


	(map-parses (lambda (x) (map-links make-lg-pair x)) sent-node)
)

(define (wrapper x)
	(score-sentence x)
	#f
)

; Loop over all atoms of type SentenceNode, processing
; each corresponding parse.
(cog-map-type wrapper 'SentenceNode)

