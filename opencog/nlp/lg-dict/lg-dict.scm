;
; lg-dict.scm
;
; Link Grammar dictionary API
;
(define-module (opencog nlp lg-dict))

(use-modules (srfi srfi-1) (opencog) (opencog nlp) (opencog exec))

(load-extension "liblg-dict" "opencog_nlp_lgdict_init")

; ---------------------------------------------------------------------

(export lg-conn-type-match?)
(set-procedure-property! lg-conn-type-match? 'documentation
"
  lg-conn-type-match? CON-A CON-B
     Return #t if connector CON-A matches CON-B, else return #f.
     Both CON-A and CON-B must be of type LgConnector.

     This does NOT check connector direction agreement; it only
     checks the connector strings, using the standard Link Grammar
     connector matching rules.
")

(export lg-conn-linkable?)
(set-procedure-property! lg-conn-linkable? 'documentation
"
  lg-conn-linkable? CON-A CON-B
     Return #t if connector CON-A can link to CON-B, else return #f.
     Both CON-A and CON-B must be of type LgConnector.

     This checks the connector strings for linkability, using the
     standard Link Grammar connector matching rules.
")

; ---------------------------------------------------------------------

(define-public (lg-dict-entry WORD)
"
  lg-dict-entry  WORD
     Fetch the dictionary entry for WORD and place it in the atomspace.
     WORD must be a WordNode. The langauge is assumed to be English.

     The dictionary entry can subsequently be obtained by calling
     (cog-incoming-by-type WORD 'LgDisjunct)
"
	(define djset (cog-incoming-by-type WORD 'LgDisjunct))
	(if (null? djset)
		(let ((dentry (LgDictEntry WORD (LgDictNode "en"))))
			(cog-execute! dentry)
			(cog-extract dentry)
			(cog-incoming-by-type WORD 'LgDisjunct)
		)
		djset)
)


(define-public (lg-get-dict-entry WORD)
"
  lg-get-dict-entry  WORD
     Fetch the dictionary entry for WORD and place it in the atomspace.
     WORD must be a WordNode.  The language is assumed to be English.

     The dictionary entry can subsequently be obtained by calling
     (cog-incoming-by-type WORD 'LgDisjunct)

     DEPRECATED! Use lg-dict-entry instead!
"
	(SetLink (lg-dict-entry WORD))
)

(define-public (lg-similar? word1 word2)
"
  lg-similar? WORD1 WORD2 - Check if two words' LG entries intersect
"
	(define (get-set w)
		(define roots (filter
			(lambda (l) (equal? (cog-type l) 'LgDisjunct))
			(cog-incoming-set w)))
		(map cog-get-partner roots (circular-list w))
	)

	; Create the dictionary entry as needed
	(lg-get-dict-entry word1)
	(lg-get-dict-entry word2)

	; Check if the two word has common LG dict entry
	(not (null? (lset-intersection equal? (get-set word1) (get-set word2))))
)

; ---------------------------------------------------------------------

(define-public (lg-conn-get-type conn)
"
  lg-conn-get-type CON - Get the LgConnectorNode out of LgConnector link
"
	(cog-outgoing-atom conn 0)
)

; ---------------------------------------------------------------------
(define-public (lg-conn-get-dir conn)
"
  lg-conn-get-dir CON - Get the LgConnDirNode out of LgConnector link
"
	(cog-outgoing-atom conn 1)
)

; ---------------------------------------------------------------------
