;
; mst-parser.scm
;
; Maximum Spanning Tree parser.
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
; The algorithm implemented is a basic maximum spanning tree algorithm.
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
(use-modules (opencog analysis))

; ---------------------------------------------------------------------
;
; Tokenize the text: take the input sentence, and return a list of the
; words in the sentence.  Words are always separated by white-space, so
; this is easy. This also makes a vague attempt to also separate commas,
; periods, and other punctuation.  Returned is a list of words.
;
(define-public (tokenize-text plain-text)
	; Prefix and suffix lists taken from the link-grammar ANY
	; language 4.0.affix file
	(define prefix "({[<«〈（〔《【［『「``„“‘'''\"…...¿¡$£₤€¤₳฿₡₢₠₫৳ƒ₣₲₴₭₺ℳ₥₦₧₱₰₹₨₪﷼₸₮₩¥៛호점†‡§¶©®℗№#")
	(define suffix ")}]>»〉）〕》】］』」’'\"%,.。:;?!‽؟？！…”–‐、～¢₵™℠")
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
				))))

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
				))))

	; Main entry point for the recursive splitter
	(define (strip-affix word) (strip-sufli word suffix-list))

	; The left-wall indicates the start of the sentence, and
	; is used to link to the head-verb, head-noun of the sentence.
	(define left-wall "###LEFT-WALL###")

	(let* ((word-list (string-split plain-text #\ ))
			(strip-list (map strip-affix word-list))
		)
		(concatenate (cons (list left-wall) strip-list))
	)
)

; ---------------------------------------------------------------------

(define-public (make-score-fn LLOBJ METHOD)
"
  make-score-fn LLOBJ METHOD -- Create a function that returns a
  score for a pair of atoms, the score being given by invoking
  METHOD on LLOBJ.  The LLOBJ must provide the METHOD, of course,
  and also the 'pair-type method, so that pairs can be assembled.

  If either atom is nil, or if the atom-pair cannot be found, then a
  default value of -1e40 is returned.
"
	; Define a losing score.
	(define bad-mi -1e40)

	(lambda (left-atom right-atom)

		; We take care here to not actually create the atoms,
		; if they aren't already in the atomspace. cog-link returns
		; nil if the atoms can't be found.
		(define wpr
			(if (and (not (null? left-atom)) (not (null? right-atom)))
				(cog-link (LLOBJ 'pair-type) left-atom right-atom)
				#f))
		(if wpr (LLOBJ METHOD wpr) bad-mi)
	)
)

; ---------------------------------------------------------------------
;
; Maximum Spanning Tree parser.
;
; Given a raw-text sentence, it splits apart the sentence into distinct
; words, and finds an (unlabelled) dependency parse of the sentence, by
; finding a dependency tree that maximizes the mutual information.
; A list of word-pairs, together with the associated mutual information,
; is returned.
;
; The M in MST normally stands for "minimum", but we want to maximize.
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
; Another idea is to apply the Dick Hudson Word Grammar landmark
; transitivity idea, but exactly how this could work for unlabelled
; trees has not been explored.
;
; So, for now, a no-links-cross constraint is handed-coded into the algo.
; Without it, it seems that the pair-MI scores alone give rather unruly
; dependencies (unclear, needs exploration).  So, in the long-run, it
; might be better to instead pick something that combines MI scores with
; mean-dependency-distance or with hubbiness. See, for example:
; Haitao Liu (2008) “Dependency distance as a metric of language
; comprehension difficulty” Journal of Cognitive Science, 2008 9(2): 159-191.
; or also:
; Ramon Ferrer-i-Cancho (2013) “Hubiness, length, crossings and their
; relationships in dependency trees”, ArXiv 1304.4086

(define-public (mst-parse-atom-seq SCORE-FN ATOM-LIST)

	; Define a losing score.
	(define bad-mi -1e30)

	; Given a list of atoms, create a numbered list of atoms.
	; The numbering provides a unique ID, needed for the graph algos.
	; i.e. if the same atom appears twice in a sequence, we need to
	; distinguish these multiple occurances.  The id does this.
	(define (atom-list->numa-list ATOMS)
		(define cnt 0)
		(map
			(lambda (ato)
				(set! cnt (+ cnt 1))
				(cons cnt ato)
			)
			ATOMS
		)
	)

	; A "numa" is a numbered atom, viz a scheme-pair (number . atom)
	;
	; Given a left-numa, and a list of numas to the right of it, pick
	; an atom from the list that has the highest-MI attachment to the
	; left atom.  Return a scheme-pair containing selected numa-pair
	; and it's cost.  Specifically, the given left-numa, and the
	; discovered right-numa, in the form ((left-numa . right-num) . mi).
	; The search is made over atom pairs scored by the SCORE-FN.
	;
	; The left-numa is assumed to be an scheme-pair, consisting of an ID,
	; and an atom; thus the atom is the cdr of the left-numa.
	; The numa-list is likewise assumed to be a list of numbered atoms.
	;
	(define (pick-best-cost-left-pair left-numa numa-list)
		(fold
			(lambda (right-numa max-pair)
				(define best-pair (first max-pair))
				(define max-mi (second max-pair))
				(define cur-mi (SCORE-FN (cdr left-numa) (cdr right-numa)))
				; Use strict inequality, so that a shorter dependency
				; length is always prefered.
				(if (< max-mi cur-mi)
					(cons (cons left-numa right-numa) cur-mi)
					max-pair
				)
			)
			(cons '() bad-mi)
			numa-list
		)
	)

	; Given a right-numa, and a list of numas to the left of it, pick
	; an atom from the list that has the highest-MI attachment to the
	; right atom.  Return a scheme-pair containing selected numa-pair
	; and it's cost.  Specifically, the given right-numa, and the
	; discovered left-numa, in the form ((left-numa . right-num) . mi).
	; The search is made over atom pairs scored by the SCORE-FN.
	;
	; The right-numa is assumed to be an scheme-pair, consisting of an ID,
	; and an atom; thus the atom is the cdr of the right-numa. The
	; numa-list is likewise assumed to be a list of numbered atoms.
	;
	(define (pick-best-cost-right-pair right-numa numa-list)
		(fold
			(lambda (left-numa max-pair)
				(define best-pair (first max-pair))
				(define max-mi (second max-pair))
				(define cur-mi (SCORE-FN (cdr left-numa) (cdr right-numa)))
				; Use less-or-equal, so that a shorter dependency
				; length is always prefered.
				(if (<= max-mi cur-mi)
					(cons (cons left-numa right-numa) cur-mi)
					max-pair
				)
			)
			(cons '() bad-mi)
			numa-list
		)
	)

	; Given a list of numas, return a costed numa-pair, in the form
	; ((left-numa . right-num) . mi).
	;
	; The search is made over atom pairs scored by the SCORE-FN.
	;
	(define (pick-best-cost-pair numa-list)

		; scan from left-most numa to the right.
		(define best-left (pick-best-cost-left-pair
				(car numa-list) (cdr numa-list)))
		(if (eq? 2 (length numa-list))
			; If the list is two numas long, we are done.
			best-left
			; else the list is longer than two numas. Pick between two
			; possibilities -- those that start with left-most numa, and
			; something else.
			(let ((best-rest (pick-best-cost-pair (cdr numa-list))))
				(if (< (second best-left) (second best-rest))
					best-rest
					best-left
				)
			)
		)
	)

	; Set-subtraction.
	; Given set-a and set-b, return set-a with all elts of set-b removed.
	; It is assumed that equal? can be used to compare elements.  This
	; should work fine for sets of ordinal-numbered atoms, and also for
	; MI-costed atom-pairs.
	(define (set-sub set rm-set)
		(filter
			(lambda (item)
				(not (any (lambda (rjct) (equal? rjct item)) rm-set))
			)
			set
		)
	)

	; Of multiple possibilities, pick the one with the highest MI
	; The choice-list is assumed to be a list of costed numa-pairs,
	; each costed pair of the form ((left-numa . right-num) . mi).
	(define (max-of-pair-list choice-list)
		; An awful numa-pair
		(define bad-pair (cons (cons (cons 0 '()) (cons 0 '())) bad-mi))

		; The tail-recursive helper that does all the work.
		(define (*pick-best choice-list best-so-far)
			(define so-far-mi (second best-so-far))
			(if (null? choice-list)
				best-so-far  ; we are done!
				(let* ((first-choice (car choice-list))
						(first-mi (second first-choice))
						(curr-best
							; use greater-than-or-equal; want to reject
							; bad-pair as soon as possible.
							(if (<= so-far-mi first-mi)
								first-choice
								best-so-far)))
					(*pick-best (cdr choice-list) curr-best)))
		)
		(*pick-best choice-list bad-pair)
	)

	; Given a single break-numa, return a list of connections to each
	; of the other numas in the sequence  The break-numa is assumed
	; to be ordinal-numbered, i.e. an integer, followed by an atom.
	; The numa-list is assumed to be a list of numas. It is presumed
	; that the brk-numa does NOT occur in the numa-list.
	;
	; The returned list is a list of costed-pairs. Each costed pair is
	; of the form ((left-numa . right-num) . mi).
	;
	; This only returns connections, if there are any. This might return
	; the empty list, if there are no connections at all.
	(define (connect-numa brk-numa numa-list)
		; The ordinal number of the break-numa.
		(define brk-num (car brk-numa))
		; The atom of the break-numa
		(define brk-node (cdr brk-numa))

		(filter-map
			(lambda (numa)
				; try-num is the ordinal number of the trial numa.
				(define try-num (car numa))
				; try-node is the actual atom of the trial numa.
				(define try-node (cdr numa))

				; Ordered pairs, the left-right order matters.
				(if (< try-num brk-num)

					; Returned value: the MI value for the pair, then the pair.
					(let ((mi (SCORE-FN try-node brk-node)))
						(if (< -1e10 mi)
							(cons (cons numa brk-numa) mi) #f))

					(let ((mi (SCORE-FN brk-node try-node)))
						(if (< -1e10 mi)
							(cons (cons brk-numa numa) mi) #f))
				)
			)
			numa-list
		)
	)

	; For each connected numbered-atom (numa), find connections between
	; that and the unconnected numas.  Return a list of MI-costed
	; connections. Each costed-connection is of the form
	; ((left-numa . right-num) . mi).
	;
	; The 'bare-numas' is a set of the unconnected atoms, labelled by
	; an ordinal number denoting sequence order.  The graph-numas is a
	; set of numas that are already a part of the spanning tree.
	; It is assumed that these two sets have no numas in common.
	;
	; This might return an empty list, if there are no connections!
	(define (connect-to-graph bare-numas graph-numas)
		(append-map
			(lambda (grph-numa) (connect-numa grph-numa bare-numas))
			graph-numas
		)
	)

	; Return true if a pair of links cross, else return false.
	(define (cross? cost-pair-a cost-pair-b)
		(define pair-a (first cost-pair-a)) ; throw away MI
		(define pair-b (first cost-pair-b)) ; throw away MI
		(define lwa (first pair-a))  ; left numa of numa-pair
		(define rwa (second pair-a)) ; right numa of numa-pair
		(define lwb (first pair-b))
		(define rwb (second pair-b))
		(define ila (car lwa))     ; ordinal number of the atom
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
			(pick-no-cross-best
				(set-sub candidates (list best)) graph-pairs)
		)
	)

	; Which numa of the pair is in the numa-list?
	(define (get-fresh cost-pair numa-list)
		(define numa-pair (first cost-pair)) ; throw away MI
		(define left-numa (first numa-pair))
		(define right-numa (second numa-pair))
		(if (any (lambda (numa) (equal? numa left-numa)) numa-list)
			left-numa
			right-numa
		)
	)

	; Find the maximum spanning tree.
	; numa-list is the list of unconnected numas, to be added to the tree.
	; graph-links is a list of edges found so far, joining things together.
	; nected-numas is a list numas that are part of the tree.
	;
	; When the numa-list becomes empty, the pair-list is returned.
	;
	; The numa-list is assumed to be a set of ordinal-numbered atoms;
	; i.e. scheme-pair of an ordinal number denoting atom-order in
	; sequwnce, and then the atom.
	;
	; The nected-numas are likewise.  It is assumed that the numa-list and
	; the nected-numas are disjoint sets.
	;
	; The graph-links are assumed to be a set of MI-costed numa-pairs.
	; That is, an float-point MI value, followed by a pair of numas.
	;
	(define (*pick-em numa-list graph-links nected-numas)

		; (format #t "----------------------- \n")
		; (format #t "enter pick-em with numalist=~A\n" numa-list)
		; (format #t "and graph-links=~A\n" graph-links)
		; (format #t "and nected=~A\n" nected-words)

		; Generate a set of possible links between unconnected numas,
		; and the connected graph. This list might be empty
		(define trial-pairs (connect-to-graph numa-list nected-numas))

		; Find the best link that doesn't cross existing links.
		(define best (pick-no-cross-best trial-pairs graph-links))

		; There is no such "best link" i.e. we've never obseved it
		; and so have no MI for it, then we are done.  That is, none
		; of the remaining numas can be connected to the existing graph.
		(if (> -1e10 (second best))
			graph-links
			(let* (

					; Add the best to the list of graph-links.
					(bigger-graph (append graph-links (list best)))

					; Find the freshly-connected numa.
					(fresh-numa (get-fresh best numa-list))
					; (jd (format #t "fresh atom=~A\n" fresh-numa))

					; Remove the freshly-connected numa from the numa-list.
					(shorter-list (set-sub numa-list (list fresh-numa)))

					; Add the freshly-connected numa to the connected-list
					(more-nected (append nected-numas (list fresh-numa)))
				)

				; If numa-list is null, then we are done. Otherwise, trawl.
				(if (null? shorter-list)
					bigger-graph
					(*pick-em shorter-list bigger-graph more-nected)
				)
			)
		)
	)

	(let* (
			; Number the atoms in sequence-order.
			(numa-list (atom-list->numa-list ATOM-LIST))

			; Find a pair of atoms connected with the largest MI
			; in the sequence.
			(start-cost-pair (pick-best-cost-pair numa-list))

			; Discard the MI.
			(start-pair (first start-cost-pair))

			; Add both of these atoms to the connected-list.
			(nected-list (list (first start-pair) (second start-pair)))

			; Remove both of these atoms from the atom-list
			(smaller-list (set-sub numa-list nected-list))
		)
		(*pick-em smaller-list (list start-cost-pair) nected-list)
	)
)

(define-public (mst-parse-text plain-text)

	; Tokenize the sentence into a list of words.
	(define word-strs (tokenize-text plain-text))

	; Create a sequence of atoms from the sequence of strings.
	(define word-list (map (lambda (str) (WordNode str)) word-strs))

	; Define where the costs are coming from.
	(define pair-obj (make-any-link-api))
	; (define pair-obj (make-clique-pair-api))

	(define mi-source (add-pair-freq-api pair-obj))

	(define scorer (make-score-fn mi-source 'pair-fmi))

	; Process the list of words.
	(mst-parse-atom-seq scorer word-list)
)

; ---------------------------------------------------------------------
; The MST parser returns a list of word-pair links, tagged with the
; mutual information between that word-pair. The functions below
; unpack each data strcture.
;
; Hmm. Should we be using goops for this?

; Get the mutual informattion of the link.
(define (mst-link-get-mi lnk) (car lnk))

; Get the left word in the link. This returns the WordNode.
(define (mst-link-get-left-word lnk)
	(define (get-pr lnk) (cadr lnk))
	(cadr (car (get-pr lnk))))

; Get the right word in the link. This returns the WordNode.
(define (mst-link-get-right-word lnk)
	(define (get-pr lnk) (cadr lnk))
	(cadr (cadr (get-pr lnk))))

; Get the word-pair of the link. This includes misc extraneous markup,
; including the word indexes in the sentence.
(define (mst-link-get-wordpair lnk)
	(ListLink (mst-link-get-left-word lnk) (mst-link-get-right-word lnk))
)

; Get the left seq-word in the link. The seq-word holds both the
; sequence number, and the word in it.
(define (mst-link-get-left-seq lnk)
	(define (get-pr lnk) (cadr lnk))
	(car (get-pr lnk)))

(define (mst-link-get-right-seq lnk)
	(define (get-pr lnk) (cadr lnk))
	(cadr (get-pr lnk)))

; Get the index number out of the sequenced word.
(define (mst-seq-get-index seq) (car seq))

; Get the word from the sequenced word.
(define (mst-seq-get-word seq) (cadr seq))

; ---------------------------------------------------------------------
;
; (use-modules (opencog) (opencog persist) (opencog persist-sql))
; (use-modules (opencog nlp) (opencog nlp learn))
; (sql-open "postgres:///en_pairs?user=linas")
;
; (fetch-all-words)
; (fetch-any-pairs)
; (mst-parse-text "faire un test")
; (mst-parse-text "Elle jouit notamment du monopole de la restauration ferroviaire")
;
; (define my-word-list (atom-list->numa-list
;      (tokenize-text "Elle jouit notamment du monopole de la restauration ferroviaire")))
; answer: du monopole 5.786411762237549
;      (tokenize-text "faire un test entre quelques mots")))
; answer: faire quelques 12.62707805633545
;      (tokenize-text "grande petit mot liste pour tout occasion")))
;
; (define my-start-pair (pick-best-cost-pair lg_any my-word-list))
