;
; parse-rank.scm
;
; Provide a score for a link-grammar parse, based on the total 
; word-pair mutual information for all word-pairs linked by
; link-grammar links. That is, every link-grammar link is looked
; up in a database of mutual information, and the total MI is
; added up for the indicated parse. It is presumed that a higher
; MI corresponds to a better parse.
;
; An alternate, possibly better(?) strategy, which is not used, is to
; obtain the maximum-spanning-tree for a given sentence, and then
; compare the link-gramar linakge to that tree. (Some initial code
; for the MST approach is below)
;
; Requires access to a database containing previously computed word-pair
; mutual information values.
;
; NOTE ON PRORGRAMMING STYLE: this file makes heavy use of callbacks.
; which is maybe not that good an idea ... its sort of a confusing
; programming style. Maybe should redo this with streams.
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
;         (WordInstanceNode "consist@79aac665-c59f-41e2-9ab6-af75b768b5c4")
;         (WordInstanceNode "mostly@69de3109-9e54-4acc-8aef-66e9da304078")
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
	; in the sentence. Get those (filter on WordInstanceNode in order
	; to ignore the SentenceNode), then map these to word nodes, 
	; and then tack them onto our list.
	(for-each 
		(lambda (x) (map-word-node add-to-word-list x))
		(cog-filter 'WordInstanceNode (cog-outgoing-set sentence))
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
; Returns #f
;
; This routine will examine each parse of the indicated sentence, and
; will assign that parse a score, based on the total mutual information
; of each of the link-grammar links in that parse.
(define (score-sentence sent-node)

	; Total score for this parse
	(define total-mi 0.0)

	; Get the list of weighted edges
	(define mi-edge-list '())
	(define (get-mi sent-link)
		(set! mi-edge-list (get-mutual-info sent-link))
		#f
	)

	; Invoke procedure 'proc' on each link-grammar link.
	; Expects as input a ParseNode (node anchoring a parse)
	; Returns whatever proc returns, as an or-map.
	(define (map-links proc parse-inst)
   	(cog-map-chase-link-dbg 'LinkGrammarLinkageLink 'EvaluationLink
      	""  " --------- linkage-found ------------ \n"
      	proc parse-inst
   	)
	)

	; Accumulate mutual info for a word pair
	; Expects as input a single link-grammar link
	; Returns #f
	; Looks up the mutual info score in the list of weighted edges,
	; and then adds this sore to 'total-mi'. This routine is meant
	; to be called from within a loop.
	;
	(define (make-lg-pair lg-link)
		(let*
			(
				; strip off EvaluationLink, and then ListLink
				(raw-pair (gadr (cog-outgoing-set lg-link)))
				(left-raw (gar raw-pair))
				(right-raw (gadr raw-pair))
				(left-node (word-inst-get-word left-raw))
				(right-node (word-inst-get-word right-raw))

				; build the actual pair of word-strings
				(word-pair
					(if (and (cog-atom? left-node) (cog-atom? right-node))
						(cons (cog-name left-node)
							(cog-name right-node))
						#f
					)
				)
				; look up the mutual info for this pair
				(score (assoc word-pair mi-edge-list))
			)
			(if score
				(set! total-mi (+ total-mi (cdr score)))
         )
		)
		#f
	)

	; Compute the total mutual information for a parse
	; Input should be a single parse-instance
	(define (score-one-parse parse-inst)
		(set! total-mi 0.0)
		(display parse-inst)
		(map-links make-lg-pair parse-inst)
		(display total-mi) (newline)
		#f
	)

	; Get the mutual information for the various word-pairs
	(for-each get-mi (cog-filter 'SentenceLink (cog-incoming-set sent-node)))

	; (display mi-edge-list) (newline)

	; Score each of the parses in the sentence
	(map-parses score-one-parse sent-node)
	#f
)

; Loop over all atoms of type SentenceNode, processing
; handing them to 'score sentence' for scoring.
(cog-map-type score-sentence 'SentenceNode)

