;
; pair-count-fr.scm
;
; Set up everything needed for the language-learning word-pair
; counting pipeline. Starts the REPL server, opens the database.
;
(use-modules (system repl common))
(use-modules (system repl server))
(use-modules (opencog) (opencog logger))
(use-modules (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))

; Write a log-file, just in case...
(cog-logger-set-filename! "/tmp/pair-count-fr.log")
(cog-logger-info "Commencez word-pair counting Francaise.")

; Tell opencog where the relex server is located.
; The port should match that in `relex-server-ady.sh`
; Note: this is the morphology split-into-two server.
(use-relex-server "127.0.0.1" 4446)

; Start the network REPL server on port 17003
(call-with-new-thread (lambda ()
	(repl-default-option-set! 'prompt "scheme@(fr-pairs)> ")
	(set-current-error-port (%make-void-port "w"))
	(run-server (make-tcp-server-socket #:port 17003)))
)

; Open the database.
; Edit the below, setting the database name, user and password.
(sql-open "odbc://linas:asdf/fr-ady")
; (sql-open "postgres:///fr_pairs?user=ubuntu&password=asdf")
