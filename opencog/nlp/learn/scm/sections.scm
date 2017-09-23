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
(define-public (get-germ-connector-seqs WORD)
"
  get-germ-connector-seqs WORD - return all connector seqeucences
  that appear in sections on the WORD. There is one connector sequence
  per section.

  Given a word, the \"germ\", assemble a list of all of the connector
  sequences that appear in sections for that germ.

  Assumes that the sections for the word are already in the atomspace.
  These can be loaded by saying (fetch-incoming-by-type WORD 'Section)
"
	; Walk over all the Sections on the word.
	; The ConnectorSeq is in position 1 in the section.
	(map (lambda (SEC) (cog-outgoing-atom SEC 1))
		(cog-incoming-by-type WORD 'Section))
)

; ---------------------------------------------------------------
;
(define-public (get-germ-connectors WORD)
"
  get-germ-connectors WORD - return all connectors that appear in
  sections on the WORD.

  Given a word, the \"germ\", assemble a list of all of the connectors
  that appear in the connector sets (disjuncts) in sections for that
  germ.

  Assumes that the sections for the word are already in the atomspace.
  These can be loaded by saying (fetch-incoming-by-type WORD 'Section)
"
	; Given a Section i.e. (word, connector-set), walk over all
	; the connectors in the connector set, and add the connector
	; to the word-list.
	(define (add-to-list SEQ WORD-LIST)
		(fold
			(lambda (CNCTR LST)
				; Hmm. Is this test for the type really needed?
				(if (eq? 'Connector (cog-type CNCTR))
					(cons CNCTR LST) LST))
			WORD-LIST
			; second atom of Section is a ConnectorSeq
			(cog-outgoing-set SEQ)))

	; Walk over all the Sections on the word.
	(delete-dup-atoms
		(fold add-to-list '() (get-germ-connector-seqs WORD)))
)

; ---------------------------------------------------------------
;
(define-public (get-germ-endpoints WORD)
"
  get-germ-endpoints WORD - return all words that appear in disjuncts.

  Given one word, the \"germ\", assemble a list of all of the words
  that appear in the connector sets (disjuncts) in sections for that
  germ.

  Assumes that the sections for the word are already in the atomspace.
  These can be loaded by saying (fetch-incoming-by-type WORD 'Section)
"
	; Walk over all the connectors, extracting the words.
	(delete-dup-atoms
		(map
			(lambda (CNCTR) (cog-outgoing-atom CNCTR 0))
			(get-germ-connectors WORD)))
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
