;
; launch-cogserver.scm
;
; Run the cogserver, needed for the language-learning disjunct
; counting pipeline. Starts the cogserver, opens the database,
; loads the database (which can take an hour or more!)
;
(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog cogserver))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (system repl common))
(use-modules (system repl server))

(add-to-load-path ".")
(load "utilities.scm")

; Get the mode, languague and database connection details
(define cog-mode (get-mode))
(define language (get-lang))
(define database-uri (get-connection-uri))

; set the prompt for the given language and mode
;(if (condicion)
(repl-default-option-set! 'prompt (string-append "scheme@(" 
    language "-" cog-mode ")> "))
;(else statement))

; Start the cogserver with configs for the given language
(start-cogserver (string-append "config/opencog-" cog-mode "-" language ".conf"))

; Open the database.
(sql-open database-uri)
(display "Opened database: ")
(display database-uri)
(display "\n")

; Load up the words and word-pairs
(display "Fetching all words from database. This may take a few minutes.\n")
(fetch-all-words)
