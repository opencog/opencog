;
; sections.scm
;
; Assorted utilities for working with sections.
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Recall what a section looks like:
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
; It should be thought of as being shaped like a spider, with a body
; at the center, and a bunch of legs. In the above, the body is the
; word "foo", and "bar" is one of the legs.  Or rather, "bar" is at
; the end of one of the legs, so that foo-bar can be though of as an
; edge connecting these two words. Its a labelled edge - the
; LgConnDirNode is the label.  Formally, the body iscalled the "germ".
;
; The utilities here include:
; get-germ-endpoints - given the germ, return a list of all endpoints
;      (legs) on all sections having that germ.
;
;
; XXX TODO - most of these utilities should be made generic, someday.
; They implement generic concepts pertaining to sections, and should
; not really be tied just to the NLP bits.
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog matrix) (opencog persist))

; ---------------------------------------------------------------
;
(define-public (get-germ-endpoints WORD)
"
  get-germ-endpoints WORD - return all words that appear in disjuncts.

  Given one word, the \"germ\", assemble a list of all of the words
  that appear in the connector sets (disjuncts) in sections for that
  germ.

  Assumes that the sections for the word are already in the atomspace.
  These can be loaded by saying
  (fetch-incoming-by-type WORD 'Section)
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

(define-public (in-gram-class? WORD GCLS)
"
  in-gram-class? WORD GRAM-CLASS - is the WORD a member of the
  grammatical class CRAM-CLASS? Returns ither #t or #f.
"
	(define memlnk (cog-link 'MemberLink WORD GCLS))
	(if (null? memlnk) #f #t)
)

; ---------------------------------------------------------------
