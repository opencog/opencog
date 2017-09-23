;
; sections.scm
;
; Assorted utilities for working with sections.
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Recall what a section looks like>
;
;     Section
;         WordNode "foo" ; or possibly a WordClassNode
;         ConnectorSeq
;             Connector
;                WordNode "bar" ; or possibly a WordClassNode
;                LgConnDirNode "+"
;             Connector
;                ....
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix) (opencog persist))

; ---------------------------------------------------------------
;
; XXX TODO - the below could probably be made generic, someday:
; given a germ, this gets all the sections on that germ, and then
; gets everything that the sections can connect to.  This is a
; generic concept ... just that the data strcutures we are using
; are not yet sufficiently generic to do this easily.
(define (get-dj-words WORD)
"
  get-dj-words WORD - return all words that appear in disjuncts.

  Given just one word, assemble a list of all of the words that
  appear in the connector sets (disjuncts) in sections for that word.
  Assumes that the sections for the word are already in the atomspace.
"
	; Given a Section i.e. (word, connector-set), walk over all
	; the connectors in the connector set, and add the word appearing
	; in the connector to the word-list. But add it only if it is not
	; already in the list.
	(define (add-to-list SEC WORD-LIST)
		(fold
			(lambda (CNCTR LST)
				(define WRD (cog-outgoing-atom CNCTR 0))
				(if ; (and
					; Is it actually a word (and not a word-class?)
					(eq? 'WordNode (cog-type WRD))
					; Is it not yet in the list?
					; Its not efficient to check here; this becomes very
					; slow for long lists - it's O(N^2)
					; (not (find (lambda (wrd) (equal? WRD wrd)) LST)))
					(cons WRD LST) LST))
			WORD-LIST
			; second atom of Section is a ConnectorSeq
			(cog-outgoing-set (cog-outgoing-atom SEC 1))))

	; Walk over all the Sections on the word.
	(delete-dup-atoms
		(fold add-to-list '() (cog-incoming-by-type WORD 'Section)))
)

; ---------------------------------------------------------------
; Predicate - is the word a member of the grammatical class, already?
(define (in-gram-class? WORD GCLS)
	(define memlnk (cog-link 'MemberLink WORD GCLS))
	(if (null? memlnk) #f #t))

; ---------------------------------------------------------------
