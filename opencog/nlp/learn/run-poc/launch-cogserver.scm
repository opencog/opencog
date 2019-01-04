;
; launch-cogserver.scm
;
; Run the cogserver, needed for the language-learning disjunct
; counting pipeline. Starts the cogserver, opens the database,
; loads the database (which can take an hour or more!)
;
(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog cogserver))
(use-modules (opencog nlp))
(use-modules (system repl common))
(use-modules (system repl server))

; NOTE
; 1. The files are loaded in pipeline order. In general, the later files
;  depend on definitions contained
; in the earlier files.
; 2. Poc changes are loaded after files of the same name are loaded so as to
; redfefine the functions.
; 3. load-from-path is used so as to be able to redfine some functions. If
; (opencog nlp learn) that will not be possilbe as some of the functions
; are not exported.
(load-from-path "opencog/nlp/learn/common.scm")
(load "redefine-common.scm")
(load-from-path "opencog/nlp/learn/utilities.scm")
(load-from-path "opencog/nlp/learn/link-pipeline.scm")
(load "redefine-link-pipeline.scm")
(load-from-path "opencog/nlp/learn/singletons.scm")
(load-from-path "opencog/nlp/learn/batch-word-pair.scm")
(load-from-path "opencog/nlp/learn/mst-parser.scm")
(load "redefine-mst-parser.scm")
(load-from-path "opencog/nlp/learn/pseudo-csets.scm")
(load-from-path "opencog/nlp/learn/shape-vec.scm")
(load-from-path "opencog/nlp/learn/summary.scm")
(load-from-path "opencog/nlp/learn/gram-class.scm")
(load-from-path "opencog/nlp/learn/gram-agglo.scm")

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
