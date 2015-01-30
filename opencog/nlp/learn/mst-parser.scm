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
; Conceptually, the graph to be spanned by the tree is a clique, with 
; with every word in the sentence being connected to every other word.
; The edge-lengths are given by the mutual information betweeen word-pairs
; (although perhaps other metrics are possible; see below).
;
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
	(define prefix "({[<«〈（〔《【［『「``„“‘'''…...¿¡$£₤€¤₳฿₡₢₠₫৳ƒ₣₲₴₭₺ℳ₥₦₧₱₰₹₨₪﷼₸₮₩¥៛호점†‡§¶©®℗№#")
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
; Given a raw-text sentence, it splits apart the sentence into distinct
; words, and finds an (unlabelled) dependency parse of the sentence, by
; finding a dependency tree that minimizes minus the mutual information.
; A list of word-pairs, together with the associated mutual information,
; is returned.
;
; There are many MST algorithms; the choice was made as follows:
; Prim is very easy; but seems too simple to give good results.
; Kruskal is good, but seems hard to control a no-link-cross constraint. (?)
; Settle on a variant of Borůvka's algo, which seems to be robust,
; and fast enough for the current needs.
;
; The no-links-cross constraint might not be required, see
; R. Ferrer-i-Cancho (2006) “Why do syntactic links not cross?”
; However, that would require changing the metric from mutual information
; to something else, perhaps incorporating the dependency distance
; (as defined by Ferrer-i-Cancho), or possibly the "hubiness", or some
; combination.  Since I really, really want to stick to entropy concepts,
; the mean-dependency-distance metric needs to be re-phrased as some
; sort of graph entropy. Hmmm...
;
; So, for now, a no-links-corss constraint is handed-coded into the algo.
; Without it, it seems that the pair-MI scores alone give rather unruly
; dependencies (unclear, needs exploration).  So, in the long-run, it
; might be better to instead pick something that combines MI scores with 
; mean-dependency-distance or with hubbiness. See, for example:
; Haitao Liu (2008) “Dependency distance as a metric of language
; comprehension difficulty” Journal of Cognitive Science, 2008 9(2): 159-191. 
; or also:
; Ramon Ferrer-i-Cancho (2013) “Hubiness, length, crossings and their
; relationships in dependency trees”, ArXiv 1304.4086

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

	; Set-subtraction. 
	; Given set-a and set-b, return set-a with all elts of set-b removed.
	; It is assumed that equal? can be used to compare elements.  This
	; should work fine for sets of ordinal-numbered words, and also for
	; MI-costed word-pairs.
	(define (set-sub set rm-set)
		(filter
			(lambda (item)
				(not (any (lambda (rjct) (equal? rjct item)) rm-set))
			)
			set
		)
	)

	; Of multiple possibilities, pick the one with the highest MI
	; The choice-list is assumed to be a list of costed word-pairs,
	; as usual: cost first, then the pair.
	(define (max-of-pair-list choice-list)
		; An awful word-pair
		(define bad-pair (list bad-mi (list (list 0 '()) (list 0 '()))))

		; The tail-recursive helper that does all the work.
		(define (*pick-best choice-list best-so-far)
			(define so-far-mi (car best-so-far))
			(if (null? choice-list)
				best-so-far  ; we are done!
				(let* ((first-choice (car choice-list))
						(first-mi (car first-choice))
						(curr-best
							; use greater-than-or-equal; want to reject
							; bad-pair as soon as possible.
							(if (<= so-far-mi first-mi)
								first-choice
								best-so-far
							)
						)
					)
					(*pick-best (cdr choice-list) curr-best)
				)
			)
		)
		(*pick-best choice-list bad-pair)
	)

	; Given a single break-word, return a list of connections to each
	; of the other words in the sentence.  The break-word is assumed
	; to be ordinal-numered, i.e. an integer, followed by a WordNode.
	; The word-list is assumed to be ordinal-numbered as well.
	; The returned list is a list of costed-pairs.
	;
	; It is presumed that the brk-word does not occur in the word-list.
	(define (connect-word lg_rel brk-word word-list)
		; The ordinal number of the break-word.
		(define brk-num (car brk-word))
		; The WordNode of the break-word
		(define brk-node (cadr brk-word))
		(map
			(lambda (word)
				; try-num is the ordinal number of the trial word.
				(define try-num (car word))
				; try-node is the actual WordNode of the trial word.
				(define try-node (cadr word))
				(if (< try-num brk-num)
					; returned value: the MI value for the pair, then the pair.
					(list (get-pair-mi lg_rel try-node brk-node)
						(list word brk-word)
					)
					(list (get-pair-mi lg_rel brk-node try-node)
						(list brk-word word)
					)
				)
			)
			word-list
		)
	)

	; For each connected word, find connections between that and the
	; unconnected words.  Return a list of MI-costed connections.
	; The 'bare-words' is a set of the unconnected words, labelled by
	; an ordinal number denoting sentence order.  The graph-words is a
	; set of words that are already a part of the spanning tree.
	; It is assumed that these two sets have no words in common.
	(define (connect-to-graph lg_rel bare-words graph-words)
		(append-map
			(lambda (grph-word) (connect-word lg_rel grph-word bare-words))
			graph-words
		)
	)

	; Return true if a pair of links cross, else return false.
	(define (cross? cost-pair-a cost-pair-b)
		(define pair-a (cadr cost-pair-a)) ; throw away MI
		(define pair-b (cadr cost-pair-b)) ; throw away MI
		(define lwa (car pair-a))  ; left word of word-pair
		(define rwa (cadr pair-a)) ; right word of word-pair
		(define lwb (car pair-b))
		(define rwb (cadr pair-b))
		(define ila (car lwa))     ; ordinal number of the word
		(define ira (car rwa))
		(define ilb (car lwb))
		(define irb (car rwb))
		(or
			; All inequalities are strict.
			(and (< ila ilb) (< ilb ira) (< ira irb))
			(and (< ilb ila) (< ila irb) (< irb ira))
		)
	)

	; Return true if the pair crosses over any pairs in the pair-list
	(define (cross-any? cost-pair cost-pair-list)
		(any (lambda (pr) (cross? pr cost-pair)) cost-pair-list)
	)

	; Find the highest-MI link that doesn't cross.
	(define (pick-no-cross-best candidates graph-pairs)
		; Despite the recursive nature of this call, we always expect
		; that best isn't nil, unless there's a bug somewhere ...
		(define best (max-of-pair-list candidates))
		(if (not (cross-any? best graph-pairs))
			best
			; Else, remove best from list, and try again.
			(pick-no-cross-best (set-sub candidates (list best)) graph-pairs)
		)
	)

	; Which word of the pair is in the word-list?
	(define (get-fresh cost-pair word-list)
		(define word-pair (cadr cost-pair)) ; throw away MI
		(define left-word (car word-pair))
		(define right-word (cadr word-pair))
		(if (any (lambda (word) (equal? word left-word)) word-list)
			left-word
			right-word
		)
	)
		
	; Find the maximum spanning tree. 
	; word-list is the list of unconnected words, to be added to the tree.
	; graph-links is a list of edges found so far, joining things together.
	; nected-words is a list words that are part of the tree.
	;
	; When the word-list become empty, the pair-list is returned.
	;
	; The word-list is assumed to be a set of ordinal-numbered WordNodes;
	; i.e. an ordinal number denoting word-order in sentence, and then
	; the word-node.
	;
	; The nected-words are likewise.  It is assumed that the word-list and
	; the nected-words are disjoint sets.
	;
	; The graph-links are assumed to be a set of MI-costed word-pairs.
	; That is, an float-point MI value, followed by a pair of words.
	; 
	(define (*pick-em lg_rel word-list graph-links nected-words)

		;(trace-msg (list "----------------------- \nenter pick-em with wordlist="
		;	word-list "\nand graph-links=" graph-links "\nand nected=" nected-words "\n"))

		; If word-list is null, then we are done. Otherwise, trawl.
		(if (null? word-list)
			graph-links
			(let* (
					; Generate a set of possible links
					(trial-pairs (connect-to-graph lg_rel word-list nected-words))
					; (ja (trace-msg (list "trial pairs=" trial-pairs "\n")))

					; Find the best link that doesn't cross existing links.
					(best (pick-no-cross-best trial-pairs graph-links))
					; (jb (trace-msg (list "best-pair=" best "\n")))

					; Add the best to the list of graph-links.
					(bigger-graph (append graph-links (list best)))

					; Find the freshly-connected word.
					(fresh-word (get-fresh best word-list))
					; (jd (trace-msg (list "fresh word=" fresh-word "\n")))

					; Remove the freshly-connected word from the word-list.
					(shorter-list (set-sub word-list (list fresh-word)))

					; Add the freshly-connected word to the cnoonected-list
					(more-nected (append nected-words (list fresh-word)))
				)
				; recurse
				(*pick-em lg_rel shorter-list bigger-graph more-nected)
			)
		)
	)

	(let* (
			; Tokenize the sentence into a list of words.
			(word-strs (tokenize-text plain-text))
			; Number the words in sentence-order.
			(word-list (str-list->numbered-word-list word-strs))
			; Find a pair of words connected with the largest MI in the sentence.
			(start-cost-pair (pick-best-cost-pair lg_any word-list))
			; Add both of these words to the connected-list.
			(nected-list (cadr start-cost-pair)) ; discard the MI
			; remove both of these words from the word-list
			(smaller-list (set-sub word-list nected-list))
		)
		(*pick-em lg_any smaller-list (list start-cost-pair) nected-list)
	)
)

; ---------------------------------------------------------------------
; (init-trace)
; (load-atoms-of-type item-type)
; (define lg_any (LinkGrammarRelationshipNode "ANY"))
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
