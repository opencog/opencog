;
; lg-dict.scm
;
; Link Grammar dictionary API
;
(define-module (opencog nlp lg-dict))

(use-modules (srfi srfi-1) (opencog) (opencog nlp))

(load-extension "liblg-dict" "opencog_nlp_lgdict_init")

; ---------------------------------------------------------------------

(export lg-dict-open)
(set-procedure-property! lg-dict-open 'documentation
"
  lg-dict-open  NAME
     Open the dictionary named in the node NAME.  The NAME must
     be a Node whose string value indicates a dictionary that can
     subsequently be found in the file system. This sets a single
     global dictionary, and all subsequent LG access routines will
     access this dictionary.
")

(export lg-dict-close)
(set-procedure-property! lg-dict-close 'documentation
"
  lg-dict-close
     Close the single global dictionary.
")

(export lg-get-dict-entry)
(set-procedure-property! lg-get-dict-entry 'documentation
"
  lg-get-dict-entry  WORD
     Fetch the dictionary entry for WORD and place it in the atomspace.
     WORD must be a WordNode.

     The dictionary entry can subsequently be obtained by calling
     (cog-incoming-by-type WORD 'LgDisjunct)
")

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
