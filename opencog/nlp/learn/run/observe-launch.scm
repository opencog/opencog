;
; observe-launch.scm
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

; Get the database connection details
(define database-uri (get-connection-uri))
(define language (get-lang))

; set the prompt for the given language
(repl-default-option-set! 'prompt (string-append "scheme@(" 
    language "-pairs)> "))

; Get the database connection details
(define database-uri (get-connection-uri))
(define language (get-lang))

; Start the cogserver with configs for the given language
(start-cogserver (string-append "opencog-" language ".conf"))

; Open the database.
(sql-open database-uri)
(display "Opened database: ")
(display database-uri)
(display "\n")
(fetch-all-words)
