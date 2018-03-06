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

(repl-default-option-set! 'prompt "scheme@(en-pairs)> ")

(add-to-load-path ".")
(load "utilities.scm")

; Get the database connection details
(define database-uri (get-connection-uri))

; Start the cogserver.
; Edit the below, setting it to the desired langauge.
; This sets the cogserver port-number and the prompt-style.
(start-cogserver "opencog-en.conf")

; Open the database.
(sql-open database-uri)
(display "Opened database: ")
(display database-uri)
(display "\n")
(display "Checking for empty database\n")
(fetch-all-words)

(define (cnt-all)
    (define cnt 0)
    (define (ink a) (set! cnt (+ cnt 1)) #f)
    (define (cnt-type x) (cog-map-type ink x))
    (map cnt-type (cog-get-types))
    cnt
)

(define atom-count (cnt-all))
(if (> atom-count 9)
    (begin
        (display "CAUTION: Database has ")
        (display atom-count)
        (display " atoms\n"))
    (begin
        (display "Database is EMPTY and ready to observe.\n")))
