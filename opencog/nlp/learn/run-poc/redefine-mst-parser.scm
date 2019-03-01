; Maximum Spanning Tree parser.
;
; Given a raw-text sentence, it splits apart the sentence into distinct
; words, and finds an (unlabelled) dependency parse of the sentence, by
; finding a dependency tree that maximizes the mutual information.
; A list of word-pairs, together with the associated mutual information,
; is returned.
;
(define-public (mst-parse-text-file plain-text mst-dist)
"
	Procedure to MST-parse sentences coming from an instance-pair weight file.
"
	(define inst-pair-pred (PredicateNode "*-Sentence Instance Pair-*"))
	(define weight-key (PredicateNode "*-Pair Weight Key-*"))

	; Return the atom holding the count, if it exists, else return nil.
	(define (get-pair-instances L-ATOM R-ATOM)
		(define maybe-list (cog-link 'ListLink L-ATOM R-ATOM))
		(if (null? maybe-list) 
			'()
			(cog-link 'EvaluationLink inst-pair-pred maybe-list)
		)
	)

	; Assuming input is tokenized, just separate by spaces
	(define (word-strs text-line)
		(string-split text-line #\ )
	)

	; Create a list of atoms from the sequence of strings and their position
	; Atoms have the structure:
	; (WordSequenceLink
	;		(WordNode "example") 
	;		(NumberNode 2)
	; )
	(define (word-list list-of-words)
		(define cnt -1)
		(map 
			(lambda (str) 
				(set! cnt (+ cnt 1))
				(WordSequenceLink
					(WordNode str)
					(NumberNode cnt)
				)
			)
			list-of-words
		)
	)	

	; Create instance-pair atom with position-tagged words
	; Also attach the corresponding weight as a value
	(define (make-pair-atom text-pair)
		(define tokens (word-strs plain-text))
		(define pair-weight (FloatValue (string->number (fifth tokens))))
		(define pare  ; Create ListLink with numbered words
			(ListLink 
				(WordSequenceLink
					(WordNode (second tokens))
					(NumberNode (first tokens))
				) 
				(WordSequenceLink
					(WordNode (fourth tokens))
					(NumberNode (third tokens))
				)
			) 
		)

		; Create EvaluationLink and assign weight
		(cog-set-value! 
			(EvaluationLink inst-pair-pred pare) 
			weight-key pair-weight
		)
	)

	; Define scoring function to look for weights in atoms
	(define scorer 
		(let ((bad-weight -1e40)) ; losing score

			; We take care here to not actually create the atoms,
			; if they aren't already in the atomspace. get-pair returns
			; nil if the atoms can't be found.
			(lambda (left-atom right-atom distance)
				(define wpr
					(if (and (not (null? left-atom)) (not (null? right-atom)))
						(get-pair-instances left-atom right-atom)
						'()
					)
				)
				(if (null? wpr) 
					bad-weight 
					(cog-value-ref (cog-value wpr weight-key) 0)
				)
			)
		)
	)

	; Assign a bad cost to links that are too long --
	; longer than 16. This is a sharp cutoff.
	; This causes parser to run at O(N^3) for LEN < 16 and
	; a faster rate, O(N^2.3) for 16<LEN. This should help.
	; Otherwise, assign modification to scorer depending on
	; mst-parsing mode. If mst-distance accounting is activated
	; shift all mi-values by 1/LEN, where LEN is the difference
	; in positions in a sentence between words in word-pair.
	(define (trunc-scorer LW RW LEN)
		(if (< 16 LEN)
			-2e25
			(let
				((modifier (if mst-dist (/ 1 LEN) 0)))
				(+ modifier (scorer LW RW LEN)))))

	; Check if blank line, which marks sentence end
	(if (equal? plain-text "")
		; Parse stored sentence and set new-sentence flag to true
		(begin
			(set! new-sent-flag #t)
			(mst-parse-atom-seq (word-list (word-strs current-sentence)) trunc-scorer)
		)

		; Check if it's the first line in a block (contains sentence to parse)
		; If not, it's an instance-pair with its weight
		(if new-sent-flag
			; store current sent, set new-sent-flag to false, flush prev instance-pairs
			(begin
				;###After parsing, flush the newly created atoms to avoid RAM explosion.
				(set! current-sentence plain-text)
				(set! new-sent-flag #f)
			)
			; if not a new-sentence, create instance-pair atom
			(make-pair-atom plain-text)
		)
	)
)


(define-public (mst-parse-text-mode plain-text cnt-mode mst-dist)

	; Tokenize the sentence into a list of words.
	(define word-strs (tokenize-text plain-text))

	; Create a sequence of atoms from the sequence of strings.
	(define word-list (map (lambda (str) (WordNode str)) word-strs))

	; Define where the costs are coming from.
	(define pair-obj
		(cond
			((or (equal? cnt-mode "clique")
				 (equal? cnt-mode "clique-dist"))
					(make-clique-pair-api))
			(else (make-any-link-api))))

	(define mi-source (add-pair-freq-api pair-obj))

	(define scorer (make-score-fn mi-source 'pair-fmi))

	; Assign a bad cost to links that are too long --
	; longer than 16. This is a sharp cutoff.
	; This causes parser to run at O(N^3) for LEN < 16 and
	; a faster rate, O(N^2.3) for 16<LEN. This should help.
	; Otherwise, assign modification to scorer depending on
	; mst-parsing mode. If mst-distance accounting is activated
	; shift all mi-values by 1/LEN, where LEN is the difference
	; in positions in a sentence between words in word-pair.
	(define (trunc-scorer LW RW LEN)
		(if (< 16 LEN)
			-2e25
			(let
				((modifier (if mst-dist (/ 1 LEN) 0)))
				(+ modifier (scorer LW RW LEN)))))

	; Process the list of words.
	(mst-parse-atom-seq word-list trunc-scorer)
)

; wrapper for backwards compatibility
(define-public (mst-parse-text plain-text)
	(mst-parse-text-mode plain-text "any" #f))

; ---------------------------------------------------------------------
(define (export-mst-parse plain-text mstparse filename)
"
  Export an MST-parse to a text file named filename,
  so that parses can be examined.
  The format is:
  [sentence]
  [word1#] [word1] [word2#] [word2]
  [word2#] [word2] [word4#] [word4]
  ...
"
	; open output file
	(define file-port (open-file filename "a"))

	; functions for getting specific parts of the link
	(define (get-mi link) (cdr link))
	(define (get-lindex link)
		(- (car (car (car link))) 1))
	(define (get-rindex link)
		(- (car (cdr (car link))) 1))
	(define (get-lword link)
		(let ((atom (cdr (car (car link)))))
			(if (cog-link? atom)
				(cog-name (gar atom))
				(cog-name atom)
			)
		)
	)
	(define (get-rword link)
		(let ((atom (cdr (cdr (car link)))))
			(if (cog-link? atom)
				(cog-name (gar atom))
				(cog-name atom)
			)
		)
	)

	; link comparator to use in sort func
	(define link-comparator
		(lambda (l1 l2)
			(< (get-lindex l1) (get-lindex l2))))

	; Print the sentence first
	(if (not (null? plain-text))
		(display
			(format #f "~a\n"
				plain-text)
			file-port))

	; Print the links if they are not bad-pair
	(for-each
		(lambda (l) ; links
			(if (> (get-mi l) -1.0e10) ; bad-MI
				(display
					(format #f "~a ~a ~a ~a ~a\n"
						(get-lindex l)
						(get-lword l)
						(get-rindex l)
						(get-rword l)
						(get-mi l))
				file-port)))
		(sort mstparse link-comparator)
	)

	; Add a new line at the end
	(display "\n" file-port)

	(close-port file-port)
)

(define-public (observe-mst-mode plain-text CNT-MODE MST-DIST EXPORT-MST)
"
  observe-mst-mode -- update pseduo-disjunct counts by observing raw text.
					  Build mst-parses using MI calculated beforehand.
					  When MST-DIST is true, word-pair MI values are adjusted
					  for distance.
					  Obtained parses are exported to file if EXPORT-MST
					  is true.

  This is the second part of the learning algo: simply count how
  often pseudo-disjuncts show up.
"
	(define file-cnt-mode (if (equal? CNT-MODE "file") #t #f))
	(define parse 
		(if file-cnt-mode
			(mst-parse-text-file plain-text MST-DIST)
			(mst-parse-text-mode plain-text CNT-MODE MST-DIST)
		)
	)

	(if (list? parse)
		(begin
			; The count-one-atom function fetches from the SQL database,
			; increments the count by one, and stores the result back
			(for-each
				(lambda (dj) (if (not (is-oversize? dj)) (count-one-atom dj)))
				(make-sections parse)
			)
			(if EXPORT-MST
				(if file-cnt-mode
					(export-mst-parse current-sentence parse "mst-parses.ull")
					(export-mst-parse plain-text parse "mst-parses.ull")
				)
			)
		)
	)
)

; Wrapper for backwards compatibility
(define-public (observe-mst plain-text)
	(observe-mst-mode plain-text "any" #f #f)
)
