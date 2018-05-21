;
; mst-parser.scm
;
; Maximum Spanning Tree parser.
;
; Copyright (c) 2014, 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below use a simple minimum spanning-tree (MST) parser to
; create an MST parse of a text sentence. This parse tree is then used
; to create a set of equivalent Link Grammar disjuncts (which are
; essentially the same thing as a local section of a sheaf of graphs;
; this is explained more, below).
;
; Input to this should be a single unicode utf8-encoded text sentence.
; It is presumed, as background, that the atomspace is loaded with a
; large number of word-pairs and their associated mutual information.
; These word-pairs need to have been previously computed.
;
; The sentence is tokenized, assuming that white-space represents word
; boundaries. Leading and trailing punctuation is stripped from each
; word, and is treated as a distinct "word".
;
; The set of words is treated as the set of vertexes of a complete graph
; or "clique", with the edges being word-pairs. The MST parser obtains
; the spanning tree that maximizes the sum of the mutual information (or
; other additive quantity) associated with the edges. This tree is the
; MST tree.
;
; After an sentence has been parsed with the MST parser, the links
; between words in the parse can be interpreted as Link Grammar links.
; There are two possible interpretations that can be given to these
; links: they are either "ANY" links, that connect between any words,
; or they are links capable of connecting ONLY those two words.
; In the later case, the link-name can be thought of as the
; concatenation of the two words it connects.
;
; In either case, one can work "backwards", and obtain the effective
; disjunct on each word, that would have lead to the given MST parse.
; For each word, this disjunct is just the collection of the other words
; that it is connected to. It is the unit-distance section of a sheaf.
;
; All the hard work is done in the `sheaf` module. This is just a very
; slim wrapper to parse the text, and update the number of times the
; disjunct has been observed.
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (srfi srfi-11))
(use-modules (opencog matrix))
(use-modules (opencog sheaf))

; ---------------------------------------------------------------------
;
; Tokenize the text: take the input sentence, and return a list of the
; words in the sentence.  It is assumed that words are always separated
; by white-space, so this is easy. This also makes a vague attempt to
; also separate commas, periods, and other punctuation.  Returned is a
; list of words.
;
; This is not terribly rigorous; it treats a somewhat arbitrary
; selection of oddball unicode punctuation marks as prefixes and
; suffixes. This list is not complete nor terribly organized; rather,
; it is built up from experience of parsing assorted texts and noting
; the kinds of stuff that actually gets used. Its slanted towards
; European languages, and may be inadequate for other languages.
;
; I did not want to get too fancy here; I want just enough to parse
; most "ordinary" text, for now.  A fancier treatment must await
; generalized handling of morphology, at which point, we can treat
; any kind of affixes, and not just punctuation.  So its kind of
; pointless to try to replace the code below by something "better",
; unless the better thing is full morphology support.
;
(define-public (tokenize-text plain-text)
	; Prefix and suffix lists taken from the link-grammar ANY
	; language 4.0.affix file
	(define prefix "({[<«〈（〔《【［『「``„“‘'''\"…..._-‐‑‒–—―¿¡$£₤€¤₳฿₡₢₠₫৳ƒ₣₲₴₭₺ℳ₥₦₧₱₰₹₨₪﷼₸₮₩¥៛호점†‡§¶©®℗№#")
	(define suffix ")}]>»〉）〕》】］』」’'\"%,.。:;?!‽؟？！…”_-‐‑‒–—―、～¢₵™℠")
	; Hey, the long-dashes below are different. So are the short dashes.
	; The first dash is the ascii-dash 0x2d.
	(define infix "-‐‑‒–—―…()[]{}")
	(define prefix-list (string->list prefix))
	(define suffix-list (string->list suffix))
	(define infix-list (string->list infix))

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
	(define (strip-prefix word)
		(if (< 0 (string-length word))
			(strip-prefli word prefix-list)
			'()))

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
	(define (strip-affix word)
		(if (< 0 (string-length word))
			(strip-sufli word suffix-list)
			'()))

	; Pad dashes with whitespace.
	; Taking string str, starting at the start-index, if it
	; contains the infx character, then pad it with a space.
	(define (pad-a-dash str infx start)
		(define idx (string-index str infx start))
		(if idx
			(let ((idp1 (+ idx 1)))
				(pad-a-dash
					(string-replace str " " idx idx)
					infx
					(+ idx 2)))
			str))

	; Pad dashes with whitespace.  Go over every character in the
	; infix-list and if it occurs in the string, put a blank space
	; in front of it.  The string splitter will then split at the
	; blank space, and the prefix stripper will do the rest.
	(define (pad-dash str ifx-list)
		(if (null? ifx-list) str
			(pad-dash (pad-a-dash str (car ifx-list) 0) (cdr ifx-list))))

	; Merge certain types of punctuation back into one.
	; e.g. three dots, or two dashes.
	(define (remerge tkl buff punct rslt)
		(if (null? tkl)
			(if (< 0 (string-length buff)) (cons buff rslt) rslt)
			(if (string=? (car tkl) punct)
				(remerge (cdr tkl) (string-append buff punct) punct rslt)
				(if (< 0 (string-length buff))
					(remerge tkl "" punct (cons buff rslt))
					(remerge (cdr tkl) "" punct (cons (car tkl) rslt))))))

	; Merge a sequence of dots back into one word.
	; Merge a sequence of ascii dashes back into one word.
	(define (remerge-dot-dash tkl)
		(remerge (remerge tkl "" "." '()) "" "-" '()))

	; The left-wall indicates the start of the sentence, and
	; is used to link to the head-verb, head-noun of the sentence.
	(define left-wall "###LEFT-WALL###")

	(let* ((pad-text (pad-dash plain-text infix-list))
			(word-list (string-split pad-text #\ ))
			(strip-list (map strip-affix word-list))
			(tok-list (concatenate (cons (list left-wall) strip-list)))
			(merge-list (remerge-dot-dash tok-list))
		)
		; (format #t "strip-list is ~A\n" strip-list)
		; (format #t "tok-list is ~A\n" tok-list)
		; (format #t "merge-list is ~A\n" merge-list)
		merge-list
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

	; Assign a bad cost to links that are too long --
	; longer than 16. This is a sharp cutoff.
	; This causes parser to run at O(N^3) for LEN < 16 and
	; a faster rate, O(N^2.3) for 16<LEN. This should help.
	(define (trunc-scorer LW RW LEN)
		(if (< 16 LEN) -2e25 (scorer LW RW LEN)))

	; Process the list of words.
	(mst-parse-atom-seq word-list trunc-scorer)
)

; ---------------------------------------------------------------------
; Return #t if the section is bigger than what the current postgres
; backend can store. Currently, the limit is atoms with at most 330
; atoms in the outgoing set.
;
; This typically occurs when the MST parser is fed a long string of
; punctuation, or a table of some kind, or other strings that are not
; actual sentences.
(define (is-oversize? SECTION)
	(< 330 (cog-arity (gdr SECTION)))
)

(define-public (observe-mst plain-text)
"
  observe-mst -- update pseduo-disjunct counts by observing raw text.

  This is the second part of the learning algo: simply count how
  often pseudo-disjuncts show up.
"
	; The count-one-atom function fetches from the SQL database,
	; increments the count by one, and stores the result back
	(for-each
		(lambda (dj) (if (not (is-oversize? dj)) (count-one-atom dj)))
		(make-sections (mst-parse-text plain-text))
	)
)

(define-public (observe-mst-extra plain-text)
"
  observe-mst-extra -- update pseduo-disjunct counts by observing raw
  text.

  See also 'observe-mst'.

  This is the second part of the learning algo: simply count how
  often pseudo-disjuncts show up. In addition to what 'observe-mst'
  does, extra atoms will be generated to preserve the MST parse
  for each of the inputs.
"
	; Tokenize the text, create WordNodes as well as WordIntanceNodes
	; for each of the words
	(define word-strs (tokenize-text plain-text))
	(define word-list (map WordNode word-strs))
	(define word-insts (map (lambda (w) (WordInstanceNode
		(random-node-name 'WordInstanceNode 36 (string-append w "@"))))
			word-strs))

	; Create a ParseNode, link each of the WordInstanceNodes with it
	; by using WordInstanceLinks
	; ReferenceLinks will also be created to link the WordInstanceNodes
	; with their corresponding WordNodes
	(define parse-node
		(ParseNode (random-node-name 'ParseNode 36 "mst_parse@")))
	(define word-inst-lks
		(map (lambda (wi) (WordInstanceLink wi parse-node)) word-insts))
	(define reference-lks
		(map (lambda (wi w) (ReferenceLink wi w)) word-insts word-list))

	; Create WordSequenceLink to record the word order
	(define num-nodes '())
	(define word-seq-lks (map
		(lambda (wi)
			(define n (NumberNode (length num-nodes)))
			(set! num-nodes (append num-nodes (list n)))
			(WordSequenceLink wi n))
		word-insts))

	; Do the MST parse
	(define mstparse (mst-parse-text plain-text))
	(define sections (make-sections mstparse))

	; Take the parse from above, replace the WordNodes with WordInstanceNodes
	; and create sections for those instances also
	(define mstparse-insts
		(map (lambda (w)
			(define l-idx (overt-get-index (wedge-get-left-overt w)))
			(define r-idx (overt-get-index (wedge-get-right-overt w)))
			(cons ; Pair this connection with the score
				(cons ; Pair the left and right word instances with their indexes
					(cons l-idx (list-ref word-insts (- l-idx 1)))
					(cons r-idx (list-ref word-insts (- r-idx 1)))
				)
				(wedge-get-score w)
			))
			mstparse))

	; Make sections using the instances
	(define section-insts (make-sections mstparse-insts))

	; The count-one-atom function fetches from the SQL database,
	; increments the count by one, and stores the result back
	(for-each
		(lambda (dj) (if (not (is-oversize? dj)) (count-one-atom dj)))
		sections
	)

	; And store the whole parse
	(for-each
		(lambda (x)
			(if (equal? (cog-type x) 'Section)
				(if (not (is-oversize? x)) (store-atom x))
				(store-atom x)))
		(append section-insts word-seq-lks word-inst-lks reference-lks)
	)

	; Export the parse to a file
	(export-mst-parses "mst-parses.txt")

	; Remove the instances and the links containing them
	(for-each cog-extract-recursive word-insts)

	; Also remove those newly generated nodes
	(for-each cog-extract num-nodes)
	(cog-extract parse-node)
)

(define-public (export-mst-parses filename)
"
  Export all the MST-parses that can be found in the
  atomspace to a text file named \"filename\", so that
  it would be easier for people to examine the parses.

  The format is:
  [sentence]
  [word1#] [word1] [word2#] [word2]
"
	(define file-port (open-file filename "a"))

	; Given a WordInstanceNode, return its word sequence number
	(define (get-index w)
		(inexact->exact (string->number
			(cog-name (word-inst-get-number w)))))

	; Given a WordInstanceNode, return its corresponding word
	(define (get-word w)
		(cog-name (word-inst-get-word w)))

	; Export a single parse
	(define (export-parse word-insts)
		; Print the sentence first
		(if (not (null? word-insts))
			(display
				(format #f "~a\n"
					(string-join (map get-word (cdr word-insts)) " "))
				file-port))

		(for-each
			(lambda (w) ; WordInstanceNodes
				(for-each
					(lambda (cs) ; ConnectorSeq
						(for-each
							(lambda (c) ; Connector
								(if (equal? (cog-name (gdr c)) "+")
									(display
										(format #f "~a ~a ~a ~a\n"
											(get-index w)
											(get-word w)
											(get-index (gar c))
											(get-word (gar c)))
										file-port)))
							(cog-outgoing-set cs)))
					(cog-chase-link 'Section 'ConnectorSeq w)))
			word-insts
		)

		; Add a new line at the end
		(display "\n" file-port)
	)

	; Export the MST-parses
	(for-each
		(lambda (p)
			(export-parse (parse-get-words-in-order p)))
		(cog-get-atoms 'ParseNode))

	(close-port file-port)
)

; ---------------------------------------------------------------------
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
