#! /usr/bin/env guile
!#
;
; Atomspace deduplication repair script
;
; Due to bugs, the SQL backend can end up with multiple copies of
; atoms. This script will find them, merge them, and sum the counts
; on the associated count truth values.  Its up to you to recompute
; anything else.
;
; This script focuses on single-words and the pairs they appear in.

(load "common.scm")

; ------------------------------------------------
(define (find-pairs wuid)
"
  find-pairs -- given a uuid of single word, find all word-pairs
  which contain the word.  The wuid must be the uuid of a WordNode.

  Returns a list of ListLink pairs that contain the WordNode.
"
	(define row #f)
	(define pair-list (list))

	; type=8 is the ListLink
	(define qry "SELECT * FROM atoms WHERE type=8")
	(dbi-query conxion qry)

	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))

		; Extract the outgoing set of the ListLink
		(let ((outset (cdr (assoc "outgoing" row))))
			(if (any (lambda (x) (eq? x wuid)) outset)
				(begin
					; (display "contains ")(display outset)(newline)
					(set! pair-list (cons outset pair-list))
				)
			)
			(set! row (dbi-get_row conxion))
		)
	)
	; Return the list of pairs
	pair-list
)

(define (find-list-link pair)
"
  Given a pair of UUID's, get the uuid of the ListLink that holds it.
  Return that UUID, else return zero
"
	(define luid 0)
	(define row #f)
	(define qry (string-append
		"SELECT * FROM atoms WHERE type=8 AND outgoing="
		(make-outgoing-str pair)))
	; (display qry)(newline)
	(dbi-query conxion qry)

	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))
		(set! luid (cdr (assoc "uuid" row)))
		(set! row (dbi-get_row conxion))
	)
	; (display "ListLink: ")(display luid)(newline)
	luid
)

(define (sum-up-eval-counts wuid auid pair-list)
"
  sum-up-eval-counts -- look for identical word-pairs
  wuid -- uuid of a word node
  auid -- uuid of another word node (holding same word)
  pair-list -- list of uuid-pairs, one of which is wuid.

  Given a list of pairs containing wuid, find the ListLinks
  correspodning tio them.  Also: find the ListLinks that hold
  the corresponding auid.  Then find the EvaluationLinks that
  correspond to these, sum up the count TV's on them and then
  delete the ListLink and EvalLink that correspond to auid.
"
	; Replace wuid by auid in the pair
	(define (replace wuid auid pair)
		(if (eq? wuid (car pair))
			(list auid (cadr pair))
			(list (car pair) auid)
		)
	)
	; Create list of pairs with the alternate pair in it
	(define alt-list (map (lambda (x) (replace wuid auid x)) pair-list))

	; Create parallel lists of UUID's of the ListLinks of the pairs
	(define luid-list (map find-list-link pair-list))
	(define laid-list (map find-list-link alt-list))

	; Get the column value on the EvalLink that holds the ListLink UUID
	(define (get-col colm uuid)
		(define row #f)
		(define val 0)
		(define qry (string-append
			"SELECT * FROM atoms WHERE type="
			EvalLinkType
			" AND outgoing="
			(make-outgoing-str (list uuid-of-any uuid))))
		; (display qry)(newline)
		(dbi-query conxion qry)

		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))
			(set! val (cdr (assoc colm row)))
			(set! row (dbi-get_row conxion))
		)
		val
	)

	; Get the stv-count on the EvalLink that holds the ListLink UUID
	(define (get-count uuid) (get-col "stv_count" uuid))

	; Get the uuid of the EvalLink that holds the ListLink UUID
	(define (get-eval-uuid uuid) (get-col "uuid" uuid))

	; Sum the counts
	; Given: luid the uuid of a ListLink containing a word pair,
	;        laid the uuid of a ListLink containing alternative word-pair
	; Then: the EvaluationLinks holding luid, laid are found,
	; the stv_counts on them are sumed,
	; the count on the evlink holding luid is updated
	; the laid and the evlink holding laid are deleted.
	(define (sum-counts luid laid)
		(if (and (< 0 luid) (< 0 laid))
			; eud contains uuid of EvaluationLink holding luid
			; aud ... etc.. holdings laid
			(let* ((eud (get-eval-uuid luid))
					(aud (get-eval-uuid laid))
				)
				(if (and (< 0 eud) (< 0 aud))
					(let* ((lcnt (get-count luid))
							(acnt (get-count laid))
							(scnt (+ lcnt acnt))
							(upd (string-append
								"UPDATE atoms SET stv_count="
								(number->string scnt)
								" WHERE uuid="
								(number->string eud)
								";"))
							(apd (string-append
								"DELETE FROM atoms WHERE uuid="
								(number->string aud)
								";"))
							(alt (string-append
								"DELETE FROM atoms WHERE uuid="
								(number->string laid)
								";"))
						)
						;(display "summo cnt ") (display lcnt)
						;(display " + ") (display acnt)
						;(display " = ") (display scnt)
						;(display " for ") (display luid)
						;(display "(") (display eud)
						;(display ") and ") (display laid)
						;(display "(") (display aud) (display ")")
						;(newline)
						;(display upd) (newline)
						(dbi-query conxion upd)
						;(display (dbi-get_status conxion)) (newline)
						(flush-query)
						;(display apd) (newline)
						(dbi-query conxion apd)
						;(display (dbi-get_status conxion)) (newline)
						(flush-query)
						;(display alt) (newline)
						(dbi-query conxion alt)
						;(display (dbi-get_status conxion)) (newline)
						(flush-query)
						scnt
					)
					;(begin
					;	(display "Missing eval id ") (display eud)
					;	(display "(for ")(display luid)
					;	(display ") and ") (display aud)
					;	(display "(for ")(display laid)
					;	(display ")")(newline)
						#f
					;)
				)
			)
			(begin
				;(display "Missing list-alt id for ") (display luid)
				;(newline)
				#f
			)
		)
	)

	; Discard alternates
	; XXX technically, this is wrong, we should be renaming these...
	(define (bug-cleanup laid)
		(if (< 0 laid)
;			(let* ((aud (get-eval-uuid laid)))
;				(if (< 0 aud)
;					(let* ((apd (string-append
;								"DELETE FROM atoms WHERE uuid="
;								(number->string aud)
;								";")))
;						(display apd) (newline)
;						(dbi-query conxion apd)
;						(display (dbi-get_status conxion)) (newline)
;						(flush-query)
;					)
;				)
;			)
			(let ((alt (string-append
						"DELETE FROM atoms WHERE uuid="
						(number->string laid)
						";")))
				(display alt) (newline)
				(dbi-query conxion alt)
				(display (dbi-get_status conxion)) (newline)
				(flush-query)
				#t
			)
			#f
		)
	)

	(define cnt-list (filter-map sum-counts luid-list laid-list))
	; (define cnt-list (filter-map bug-cleanup laid-list))

	; (display alt-list) (newline)
	(display "num of word-pairs: ") (display (length pair-list))(newline)
	(display "num of alt-pairs: ") (display (length alt-list))(newline)
	(display "num luids (ListLinks): ") (display (length luid-list))(newline)
	(display "num of laids (listLinks holding alts): ") (display (length
		(filter (lambda (x) (< 0 x)) laid-list)))(newline)
	(display "num actually fixed up: ") (display (length cnt-list))(newline)
	(newline)
)

(define (swap-alts altid wantid pair-list)
"
  swap-alts -- fix up the uuids -- replace altid by wantid in
  the ListLink that contains the altid.  This modifies the
  ListLink; it usually should not create any duplicates, as
  long as the pair-list was previously de-duped. However, for
  multiple lists, this may be violated :-(

  altid is the uuid of the unwanted WordNode
  wantid is the uuid of the correct WordNode
  pair-list is the list of uuid-pairs, one of which is altid.
"
	; Replace wuid by auid in the pair
	(define (replace wuid auid pair)
		(if (eq? wuid (car pair))
			(list auid (cadr pair))
			(list (car pair) auid)
		)
	)
	; The laid-list is the list of ListLinks that contain the
	; wrong word UUID.
	(define laid-list (map find-list-link pair-list))

	; Create list of pairs with the fixed pair in it
	(define fix-list (map (lambda (x) (replace altid wantid x)) pair-list))

	; fixup ListLink laid by inserting fix into it
	(define (fixup laid fix)
		(define upd (string-append
			"UPDATE atoms SET outgoing="
			(make-outgoing-str fix)
			" WHERE uuid="
			(number->string laid)
			";"))
		;(display upd) (newline)
		(dbi-query conxion upd)
		;(display (dbi-get_status conxion)) (newline)
		(flush-query)
	)

	(for-each fixup laid-list fix-list)

	(display "Num of relabeled ListLinks: ")
	(display (length laid-list)) (newline)
	(newline)
	(flush-output-port (current-output-port))
)

;(define pair-list (find-pairs 6844))
;(display "Found word pairs: ") (display (length pair-list))(newline)
;(sum-up-eval-counts 6844 27942 pair-list)

;(define alt-list (find-pairs 27942))
;(display "Found alt pairs: ") (display (length alt-list))(newline)
;(swap-alts 27942 6844 alt-list)

;(define pair-list (find-pairs 42673))
;(display "Found word pairs: ") (display (length pair-list))(newline)
;(sum-up-eval-counts 42673 367746 pair-list)

;(define alt-list (find-pairs 367746))
;(display "Found alt pairs: ") (display (length alt-list))(newline)
;(swap-alts 367746 42673 alt-list)

;(define pair-list (find-pairs 28019))
;(display "Found word pairs: ") (display (length pair-list))(newline)
;(sum-up-eval-counts 28019 270920 pair-list)

;(define alt-list (find-pairs 270920))
;(display "Found alt pairs: ") (display (length alt-list))(newline)
;(swap-alts 270920 28019 alt-list)
; --------------------------------------------------------------

(define (escape-quote word)
"
  If word has a single-quote in it, then escape it!
"
	(define qk (string-index word #\'))
	(if qk
		(string-replace word "''" qk (+ qk 1))
		word
	)
)

(define (get-word-uuids word)
"
  Given a word (string), get the UUID's of all WordNodes holding
  that word. Return these as a list.
"
	(define row #f)
	(define wuid-list (list))
	(define qry (string-append
		"SELECT uuid FROM atoms WHERE type=" WordNodeType
			" AND name='" (escape-quote word) "';"))
	(display qry)(newline)
	(dbi-query conxion qry)

	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))
		(set! wuid-list (cons (cdr (assoc "uuid" row)) wuid-list))
		(set! row (dbi-get_row conxion))
	)
	(display "Word ")(display word)
	(display " has uuids ")(display wuid-list) (newline)
	wuid-list
)

(define (sum-word-counts word)
"
  Given a word (string), sum up the stv-count of all WordNodes holding
  that word. Return the sum
"
	(define row #f)
	(define sum 0)
	(define qry (string-append
		"SELECT uuid,stv_count FROM atoms WHERE type=" WordNodeType
			" AND name='" (escape-quote word) "';"))
	(display qry)(newline)
	(dbi-query conxion qry)

	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))
		(set! sum (+ (cdr (assoc "stv_count" row)) sum))
		(set! row (dbi-get_row conxion))
	)
	(display "Word ")(display word)
	(display " has sum ")(display sum) (newline)
	sum
)

; --------------------------------------------------------------

(define duplicate-word-list
	(look-for-dupes (string-append
		"SELECT uuid, name FROM atoms WHERE type="
		WordNodeType) "name"))

(display "The duplicate word list is: ")
(display duplicate-word-list) (newline)
(flush-output-port (current-output-port))

;(define dup-wuid-lists
;	(map get-word-uuids duplicate-word-list))

(define (dedupe-word word)
"
  Deduplicate the word. This sums up counts on the EvaluationLinks
  that contain the duplicted words. It also relabels the ListLinks
  that use the wrong word uid.  Basically it does everything except
  to sum up the WordNodes themselves.
"
	; First, get a list of the uuid's associated with the word.
	(define wuid-list (get-word-uuids word))

	; Next, find the smallest uuid in that list
	(define (get-smallest-uuid uuid-list)
		(define sma 6123456123123)
		(for-each (lambda (x) (if (< x sma) (set! sma x))) uuid-list)
		(display "smallest wuid is ")(display sma)(newline)
		sma
	)
	(define smallest-wuid (get-smallest-uuid wuid-list))

	; Next, remove the smallest uuid from the list  ...
	(define bad-wuid-list
		(remove (lambda (x) (eq? x smallest-wuid)) wuid-list))

	; Get a list of all word-pairs that have the smallest-wuid in it.
	; This is a list of uuid-pairs; these uuid-pairs occur in some
	; ListLink somewhere.
	(define wuid-word-pairs (find-pairs smallest-wuid))

	; Fixup ListLinks holding the bad WordNode.
	; This sums and removes duplicates, and relabels the rest.
	; This is the final, mster routine.
	(define (fixup-bad bad)
		(define alt-list (list))
		; First, remove duplicates
		(sum-up-eval-counts smallest-wuid bad wuid-word-pairs)
		; Next, relabel the rest
		(set! alt-list (find-pairs bad))
		(display "Need to relabel bad pairs: ")
		(display (length alt-list))(newline)
		(swap-alts bad smallest-wuid alt-list)
		(flush-output-port (current-output-port))
	)

	(display "Number of word-pairs: ")
	(display (length wuid-word-pairs))(newline) (newline)
	(flush-output-port (current-output-port))

	; Finally, walk over the bad-wuid-list, and dupe each of these,
	; one at a time, to the smallest wuid.
	(display "The bad wuids: ")(display bad-wuid-list)(newline)
	(for-each fixup-bad bad-wuid-list)
)

;(for-each dedupe-word duplicate-word-list)

; --------------------------------------------------------------
; Finally, sum up the word-counts.

(define (dedupe-sum-word-counts word)
"
  sum-word-counts -- Given the (string) word, find all WordNodes that
  hold this word, sum up the counts on all of them, assign the count to
  the one with the smallest uuid, and delete the rest.
"
	; First, get a list of the uuid's associated with the word.
	(define wuid-list (get-word-uuids word))

	; Next, find the smallest uuid in that list
	(define (get-smallest-uuid uuid-list)
		(define sma 6123456123123)
		(for-each (lambda (x) (if (< x sma) (set! sma x))) uuid-list)
		(display "smallest wuid is ")(display sma)(newline)
		sma
	)
	(define smallest-wuid (get-smallest-uuid wuid-list))

	; Next, remove the smallest uuid from the list  ...
	(define bad-wuid-list
		(remove (lambda (x) (eq? x smallest-wuid)) wuid-list))

	; Next, sum up the counts
	(define sum-of-counts (sum-word-counts word))

	; Next, update the master count
	(define upd (string-append
		"UPDATE atoms SET stv_count="
		(number->string sum-of-counts)
		" WHERE uuid="
		(number->string smallest-wuid)
		";"))

	(display upd)(newline)
	(dbi-query conxion upd)
	(display (dbi-get_status conxion)) (newline)
	(flush-query)

	; Now, delete the bad words.
	(delete-atoms bad-wuid-list 0 #t)
)

(for-each dedupe-sum-word-counts duplicate-word-list)
