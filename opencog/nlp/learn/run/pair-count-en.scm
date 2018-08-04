;
; pair-count-en.scm
;
; Set up everything needed for the language-learning word-pair
; counting pipeline. Starts the CogServer, opens the database.
;
(use-modules (system repl common))
(use-modules (system repl server))
(use-modules (opencog) (opencog logger))
(use-modules (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog cogserver))

(repl-default-option-set! 'prompt "scheme@(en-pairs)> ")

;;; Start the network REPL server on port 17005
;;; Unfortunately, this runs about 3x slower than the cogserver,
;;; when establishing new connections, and that's bad.
;;;
;;; Write a log-file, just in case...
;;;(cog-logger-set-filename! "/tmp/pair-count-en.log")
;;;(cog-logger-info "Start word-pair counting for English.")
;;;(call-with-new-thread (lambda ()
;;;	(set-current-error-port (%make-void-port "w"))
;;;	(run-server (make-tcp-server-socket #:port 17005)))
;;;)

; Start the cogserver on port 17005
(start-cogserver "config/opencog-pairs-en.conf")

; Open the database.
; Edit the below, setting the database name, user and password.
(sql-open "postgres:///en_pairs?user=ubuntu&password=asdf")
