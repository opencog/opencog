;
; compute-mi.scm
; Written by glicerico, March 2018
;
; Computes the mutual information for the word pairs.
; Takes as input the counting mode: lg, clique, clique-distance
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

(define (comp-mi cnt-mode)
  (define pair-obj '())
  (define star-obj '())
  (cond
    ((equal? cnt-mode "lg")
      (set! pair-obj (make-any-link-api)))

    ((or (equal? cnt-mode "clique")
         (equal? cnt-mode "clique-dist"))
        (set! pair-obj (make-clique-pair-api))))

  (set! star-obj (add-pair-stars pair-obj))
  (batch-pairs star-obj)
  
  ; Print the sql stats
  (print-matrix-summary-report star-obj)

  ; Clear the sql cache and the stats counters
  (sql-close)
 
  (display "Done computing MI from word pairs.\n")
)

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
