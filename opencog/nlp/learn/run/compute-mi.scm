;
; Run the cogserver, needed for the language-learning disjunct
; counting pipeline. Starts the cogserver, opens the database,
; loads the database (which can take an hour or more!)
;
(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog matrix))
(use-modules (opencog cogserver))
(use-modules (opencog nlp) (opencog nlp learn))

(add-to-load-path ".")
(load "utilities.scm")

; Get the database connection details
(define database-uri (get-connection-uri))

; Open the database.
(sql-open database-uri)

(define ala (make-any-link-api))
(define asa (add-pair-stars ala))
(batch-pairs asa)
(print-matrix-summary-report asa)

; FIXME Is an sql-store needed?
(sql-close)

(display "Done\n")