;
; mst-parser.scm
;
; Minimum Spanning Tree parser.
;
; Copyright (c) 2014 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below implement a simple spanning-tree parser. The goal
; of this parser is to learn a base set of link-grammar disjuncts.
;
; Input to this should be a single sentence. It is presumed, as
; background, that a large number of word-pairs with associated mutual
; information is already avaialable from the atomspace (obtained
; previously).
;
; The algorithm implemented is a basic minimum spanning tree algorithm.
; Initially, a graph clique is constructed from the words in the
; sentence, with every word connected to every other word. The mutual
; information betweeen wrd-pairs is used to determine graph edge lengths.
; The spanning tree is then obtained. Finally, disjuncts are created from
; the resulting parse, by looking at how each word is attached to the
; other words.  The disjuncts are then recorded.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (srfi srfi-11))

; ---------------------------------------------------------------------
;
; Tokenize the text: take the input sentence, and return a list of the
; words in the sentence.  Words are always separated by white-space, so
; this is easy. This also makes a vague attempt to also separate commas,
; periods, and other punctuation.  Returned is a list of words.
;
(define (tokenize-text plain-text)
	; Prefix and suffix lists taken from the link-grammar ANY
	; language 4.0.affix file
	(define prefix "({[<«〈（〔《【［『「``„“‘'''…...¿¡$£₤€¤₳฿₡₢₠₫৳ƒ₣₲₴₭₺ℳ₥₦₧₱₰₹₨₪﷼₸₮₩¥៛호점†††‡§¶©®℗№#")
	(define suffix ")}]>»〉）〕》】］』」’'%,.。:;?!‽؟？！…”–‐、～¢₵™℠")
	(define prefix-list (string->list prefix))
	(define suffix-list (string->list suffix))

	; Tail-recursive helper function. Strip off any prefixes appearing
	; in the list prefli, and return a list of the stripped prefixes,
	; and the remaining word.
	(define (strip-prefli word prefli)
		(if (null? prefli)
			(list word)  ; prefix list is empty, return the word.
			(let* ((punct (car prefli))
					(head (string-ref word 0)))
				(if (eq? punct head) ; it there's a match, then split
					(append
						(list (string punct))
						(strip-prefix (substring word 1)) ; loop again.
					)
					(strip-prefli word (cdr prefli)) ; if no match, recurse.
				)
			)
		)
	)
	; Main entry point for the recursive prefix splitter
	(define (strip-prefix word) (strip-prefli word prefix-list))

	; Tail-recursive helper function. Strip off any suffixes appearing
	; in the list sufli, and return a list of the stripped prefixes,
	; the remaining word, and the stripped suffixes.
	(define (strip-sufli word sufli)
		(if (null? sufli)
			(strip-prefix word)
			(let* ((punct (car sufli))
					(len (- (string-length word) 1))
					(tail (string-ref word len)))
				(if (eq? punct tail)
					(append
						(strip-affix (substring word 0 len))
						(list (string punct))
					)
					(strip-sufli word (cdr sufli))
				)
			)
		)
	)
	; Main entry point for the recursive splitter
	(define (strip-affix word) (strip-sufli word suffix-list))

	(let* ((word-list (string-split plain-text #\ )))
		(concatenate (map strip-affix word-list))
	)
)

; ---------------------------------------------------------------------
; Return the mutual information for a pair of words.
;
; The pair of words are presumed to be connected by the relationship
; lg_rel.  The left and right words are presumed to be WordNodes, or nil.
; If either word is nill, or if the word-pair cannot be found, then a
; default value of -1000 is returned.

(define (get-pair-mi lg_rel left-word right-word)

	; Define a losing score.
	(define bad-mi -1000)

	; We take care here to not actually create the atoms,
	; if they aren't already in the atomspace. cog-node returns
	; nil if the atoms can't be found.
	(define wpr 
		(if (and (not (null? left-word)) (not (null? right-word)))
			(cog-link 'ListLink left-word right-word)
			'()))
	(define evl
		(if (not (null? wpr))
			(cog-link 'EvaluationLink lg_rel wpr)
			'()))
	(if (not (null? evl))
		(tv-conf (cog-tv evl))
		bad-mi
	)
)

; ---------------------------------------------------------------------
; Return the mutual information for a pair of words.
;
; The pair of words are presumed to be connected by the relationship
; lg_rel.  The left and right words are presumed to be strings.
; If the word-pair cannot be found, then a default value of -1000 is
; returned.

(define (get-pair-mi-str lg_rel left-word-str right-word-str)

	(get-pair-mi lg_rel
		(cog-node 'WordNode left-word-str)
		(cog-node 'WordNode right-word-str)
	)
)

; ---------------------------------------------------------------------
;
; Minimum Spanning Tree parser.
;
; Choice of algorithms:
; Prim is very easy; but seems too easy to give good results.
; Kruskal is good, but seems hard to control a no-link-cross contraint.
; Settle on a variant of Borůvka's algo.
; A no-links-cross constraint is not required, see
; R. Ferrer-i-Cancho (2006) “Why do syntactic links not cross?”

(define (mst-parse-text plain-text)
	(define lg_any (LinkGrammarRelationshipNode "ANY"))

	; Define a losing score.
	(define bad-mi -1000)

	; Given a list of strings, create a numbered list of word-nodes.
	; The numbering provides a unique ID, needed for the graph algos.
	(define (str-list->numbered-word-list word-strs)
		(define cnt 0)
		(map
			(lambda (str)
				(set! cnt (+ cnt 1))
				(list cnt (cog-node 'WordNode str))
			)
			word-strs
		)
	)
	; Given a left-word, and a list of words to the right of it, pick
	; a word from the list that has the highest-MI attachment to the left
	; word.  Return a list containing that cost and the selected word-pair
	; (i.e. the specified left-word, and the chosen right-word).
	; The search is made over word pairs united by lg_rel.
	;
	; The left-word is assumed to be an list, consisting of an ID, and
	; a WordNode; thus the WordNode is the cadr of the left-word.
	; The word-list is likewise assumed to be a list of numbered WordNodes.   
	;
	; to-do: might be better to use values for the return value....
	(define (pick-best-cost-left-pair lg_rel left-word word-list)
		(fold
			(lambda (right-word max-pair)
				(define max-mi (car max-pair))
				(define best-pair (cdr max-pair))
				(define cur-mi (get-pair-mi lg_rel (cadr left-word) (cadr right-word)))
				; Use strict inequality, so that a shorter dependency
				; length is always prefered.
				(if (< max-mi cur-mi)
					(list cur-mi (list left-word right-word))
					max-pair
				)
			)
			(list bad-mi '())
			word-list
		)
	)

	; Given a right-word, and a list of words to the left of it, pick
	; a word from the list that has the highest-MI attachment to the 
	; right word.  Return a list containing that cost and the selected
	; word-pair (i.e. the chosen left-word, and the specified right-word).
	; The search is made over word pairs united by lg_rel.
	;
	; The right-word is assumed to be an list, consisting of an ID, and
	; a WordNode; thus the WordNode is the cadr of the right-word.
	; The word-list is likewise assumed to be a list of numbered WordNodes.   
	;
	; to-do: might be better to use values for the return value....
	(define (pick-best-cost-right-pair lg_rel right-word word-list)
		(fold
			(lambda (left-word max-pair)
				(define max-mi (car max-pair))
				(define best-pair (cdr max-pair))
				(define cur-mi (get-pair-mi lg_rel (cadr left-word) (cadr right-word)))
				; Use less-or-equal, so that a shorter dependency
				; length is always prefered.
				(if (<= max-mi cur-mi)
					(list cur-mi (list left-word right-word))
					max-pair
				)
			)
			(list bad-mi '())
			word-list
		)
	)

	; Given a list of words, return a list containing the cost and the
	; word-pair with the best cost (highest MI, in this case). The search
	; is made over word pairs united by lg_rel.
	;
	; to-do: might be better to use values for the return value....
	(define (pick-best-cost-pair lg_rel word-list)

		; scan from left-most word to the right.
		(define best-left (pick-best-cost-left-pair lg_rel (car word-list) (cdr word-list)))
		(if (eq? 2 (length word-list))
			; If the list is two words long, we are done.
			best-left
			; else the list is longer than two words. Pick between two
			; possibilities -- those that start with left-most word, and
			; something else.
			(let ((best-rest (pick-best-cost-pair lg_rel (cdr word-list))))
				(if (< (car best-left) (car best-rest))
					best-rest
					best-left
				)
			)
		)
	)

	; Given a cost-word-pair (mi & numbered-word-pair) and a word-list
	; remove both words of the pair from the word-list. Return the
	; trimmed word-list.
	(define (trim-list cost-word-pair word-list)
		(let ((left-word (car (cadr cost-word-pair)))
				(right-word (cadr (cadr cost-word-pair)))
			)
			(filter
				(lambda (word)
					(and
						(not (equal? word left-word))
						(not (equal? word right-word))
					)
				)
				word-list
			)
		)
	)

	; Split the list into two lists: those stictly to the left, and 
	; strictly to the right of the break-word.
	(define (split-list brk-word word-list)
		(let-values (
				((left-list right-list)
					(break (lambda (word) (equal? word brk-word)) word-list))
			)
			; XXX right-list should never be null, because brk-word
			; should always appear in the list. Should we throw??
			(if (null? right-list)
				(list left-list '())
				(list left-list (cdr right-list))
			)
		)
	)

	; remove the given word from the list
	(define (cut-from-list cut-word word-list)
		(remove (lambda (word) (equal? word cut-word)) word-list)
	)

	; This is almost right but not quite; its only allowing
	; connections to the pair, when it should allow connections 
	; to any in the connected-vertex list.  Argh.  Tomorrow.
	;
	(define (*pick-em lg_rel word-list best-pair pair-list)

		; Of multiple possibilities, pick the one with the highest MI
		; The choice-list is assumed to be a list of costed word-pairs,
		; as usual: cost first, then the pair.
		(define (pick-best choice-list best-so-far)
			(define so-far-mi (car best-so-far))
			(if (null? choice-list)
				best-so-far  ; we are done!
				(let* ((first-choice (car choice-list))
						(first-mi (car first-choice))
						(curr-best
							(if (< so-far-mi first-mi)
								first-choice
								best-so-far
							)
						)
					)
					(pick-best (cdr choice-list) curr-best)
				)
			)
		)

		; If word-list is null, then we are done.
		(if (null? word-list)
			(append pair-list (list best-pair))
			(let* (
					; list of solutions
					(soln-list (append pair-list (list best-pair)))

					; The two words of the input word-pair.
					(left-word (car (cadr start-pair)))
					(right-word (cadr (cadr start-pair)))

					; split the list into two, using the left-word to break
					(l-split (split-list left-word word-list))
					(ll-list (car l-split))
					(lr-list (cut-from-list right-word (cadr l-split)))

					; split the list into two, using the right-word to break
					(r-split (split-list right-word word-list))
					(rl-list (cut-from-list left-word (car r-split)))
					(rr-list (cadr r-split))

					; Find the best links that attach to either side
					; the left-word becomes the right-most word for the ll-list
					(best-ll (pick-best-cost-right-pair lg_rel left-word ll-list))
					; the left-word becomes the left-most word for the lr-list
					(best-lr (pick-best-cost-left-pair lg_rel left-word lr-list))

					; the right-word becomes the right-most word for the rl-list
					(best-rl (pick-best-cost-right-pair lg_rel right-word rl-list))
					; the right-word becomes the left-most word for the rr-list
					(best-rr (pick-best-cost-left-pair lg_rel right-word rr-list))

					; Define the next-best link
					(next-best
						(pick-best
							(list best-ll best-lr best-rl best-rr)
							(list bad-mi (list (list 0 '()) (list 0 '())))
						)
					)
					(trimmed-list (trim-list start-pair word-list))
				)
				; recurse
				(pick-em lg_rel trimmed-list next-best soln-list)
			)
		)
	)


	(define (do-it lg_rel word-list)
		(define start-pair (pick-best-cost-pair lg_rel word-list))
		(*pick-em lg_rel word-list start-pair '())
	)

	(let* ((word-strs (tokenize-text plain-text))
			(word-list (str-list->numbered-word-list word-strs))
			(start-cost-pair (pick-best-cost-pair lg_any word-list))
		)
(display start-cost-pair)
	)
)

; (define lg_any (LinkGrammarRelationshipNode "ANY"))
; (init-trace)
; (load-atoms-of-type item-type)
; (fetch-incoming-set lg_any)
; (mst-parse-text "faire un test")
; (mst-parse-text "Elle jouit notamment du monopole de la restauration ferroviaire")
;
; (define my-word-list (str-list->numbered-word-list
;      (tokenize-text "Elle jouit notamment du monopole de la restauration ferroviaire")))
; answer: du monopole 5.786411762237549
;      (tokenize-text "faire un test entre quelques mots")))
; answer: faire quelques 12.62707805633545
;      (tokenize-text "grande petit mot liste pour tout occasion")))
;
; (define my-start-pair (pick-best-cost-pair lg_any my-word-list))
; (split-list (cadr my-start-pair) my-word-list)
; (prr lg_any (cadr my-start-pair) my-word-list)
