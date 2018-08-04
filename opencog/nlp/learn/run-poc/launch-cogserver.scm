;
; launch-cogserver.scm
;
; Run the cogserver, needed for the language-learning disjunct
; counting pipeline. Starts the cogserver, opens the database,
; loads the database (which can take an hour or more!)
;
; If run in mst-mode it also fetches the word-pairs
;
(use-modules (system repl common))
(use-modules (system repl server))
(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog) (opencog logger))
(use-modules (opencog cogserver))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog matrix))

(add-to-load-path ".")
(load "utilities.scm")

; Get the mode, languague and database connection details
(define cog-mode (get-mode))
(define language (get-lang))
(define database-uri (get-connection-uri))

; Set the prompt for the given language and mode
(repl-default-option-set! 'prompt (string-append "scheme@(" 
    language "-" cog-mode ")> "))

; Start the cogserver with configs for the given language
(start-cogserver (string-append "config/opencog-" cog-mode "-" language ".conf"))

; Open the database.
(sql-open database-uri)
(display "Opened database: ")
(display database-uri)
(display "\n")

; Load up the words from the database
(display "Fetching all words from database. This may take a few minutes.\n")
(fetch-all-words)

; NOTE: This script assumes that compute-mi-launch.scm has
; already stored the mutual information for the pairs after
; an observe pass over the same sentences to be parsed.
(define pair-obj '())
(define star-obj '())

(if (equal? "mst" cog-mode)

	(begin
		; Load up the word-pairs -- this can take over half an hour!
		(display "Fetch all word-pairs. This may take a few minutes!\n")

		; The object which will be providing pair-counts for us.
		; We can also do MST parsing with other kinds of pair-count objects,
		; for example, the clique-pairs, or the distance-pairs.
		(set! pair-obj (make-any-link-api))
		(set! star-obj (add-pair-stars pair-obj))
		(pair-obj 'fetch-pairs)

		; Print the sql stats
		(sql-stats)

		; Clear the sql cache and the stats counters
		(sql-clear-cache)
		(sql-clear-stats)
		(print-matrix-summary-report star-obj)

		(display "Done fetching pairs.\n")))
