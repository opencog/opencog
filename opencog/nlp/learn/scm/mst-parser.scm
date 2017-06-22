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
; The scripts below use a simple spanning-tree (MST) parser to create
; MST parse of a sentence. The goal of this parse is to create a base
; set of link-grammar disjuncts.
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
; European langauges, and may be inadequate for other langauges.
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

	; Process the list of words.
	(mst-parse-atom-seq word-list scorer)
)

; ---------------------------------------------------------------------

; Return the word-pair of the mst-link, as a listLink of WorNodes.
(define (mst-link-get-wordpair lnk)
	(ListLink (mst-link-get-left-atom lnk) (mst-link-get-right-atom lnk))
)

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
