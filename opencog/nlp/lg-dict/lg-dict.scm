(setenv "LTDL_LIBRARY_PATH"
    (if (getenv "LTDL_LIBRARY_PATH")
        (string-append (getenv "LTDL_LIBRARY_PATH")
            ":/usr/local/lib/opencog:/usr/local/lib/opencog/modules")
        "/usr/local/lib/opencog:/usr/local/lib/opencog/modules"))

(define-module (opencog nlp lg-dict))

(use-modules (srfi srfi-1) (opencog) (opencog nlp))

(load-extension "liblg-dict" "opencog_nlp_lgdict_init")

; ---------------------------------------------------------------------
(define-public (lg-similar? word1 word2)
"
  lg-similar? - Check if two words' LG entries intersect
"
	(define (get-set w)
 		(define roots (filter (lambda (l) (equal? (cog-type l) 'LgDisjunct)) (cog-incoming-set w)))
		(map cog-get-partner roots (circular-list w))
	)

	; create the dictionary entry as needed
	(lg-get-dict-entry word1)
	(lg-get-dict-entry word2)
	; check if the two word has common LG dict entry
	(not (null? (lset-intersection equal? (get-set word1) (get-set word2))))
)

; ---------------------------------------------------------------------
(define-public (lg-conn-get-type conn)
"
  lg-conn-get-type - Get the LgConnectorNode out of LgConnector link
"
	(car (cog-outgoing-set conn))
)

; ---------------------------------------------------------------------
(define-public (lg-conn-get-dir conn)
"
  lg-conn-get-dir - Get the LgConnDirNode out of LgConnector link
"
	(cadr (cog-outgoing-set conn))
)

; ---------------------------------------------------------------------
