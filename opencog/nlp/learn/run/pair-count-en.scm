;
; pair-count-en.scm
;
; Set up everything needed for the language-learning word-pair
; counting pipeline. Starts the REPL server, opens the database.
;
(use-modules (system repl server))
(use-modules (opencog) (opencog logger))
(use-modules (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))

; Write a log-file, just in case...
(cog-logger-set-filename! "/tmp/pair-count-en.log")

; Tell opencog where the relex server is located.
; The port should match that in `relex-server-any.sh`
(use-relex-server "127.0.0.1" 4445)

; Start the network REPL server on port 17005
(call-with-new-thread (lambda ()
	(set-current-error-port (%make-void-port "w"))
	(run-server (make-tcp-server-socket #:port 17005)))
)

; Open the database.
; Edit the below, setting the database name, user and password.
(sql-open "postgres:///en_pairs?user=ubuntu&password=asdf")
