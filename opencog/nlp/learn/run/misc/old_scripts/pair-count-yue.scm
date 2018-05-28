;
; pair-count-yue.scm
;
; Set up everything needed for the language-learning word-pair
; counting pipeline. Starts the REPL server, opens the database.
;
(use-modules (system repl common))
(use-modules (system repl server))
(use-modules (opencog) (opencog logger))
(use-modules (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog cogserver))

(repl-default-option-set! 'prompt "scheme@(yue-pairs)> ")

; Tell opencog where the relex server is located.
; The port should match that in `relex-server-any.sh`
; Not using RelEx any longer.
; (use-relex-server "127.0.0.1" 4445)

;;; Start the network REPL server on port 17006
;;; Unfortunately, this runs about 3x slower than the cogserver,
;;; when establishing new connections, and that's bad.
;;;
;;; Write a log-file, just in case...
;;;(cog-logger-set-filename! "/tmp/pair-count-yue.log")
;;;(cog-logger-info "Start word-pair counting for 廣東話 Cantonese.")
;;;
;;;; Start the network REPL server on port 17006
;;;(call-with-new-thread (lambda ()
;;;	(set-current-error-port (%make-void-port "w"))
;;;	(run-server (make-tcp-server-socket #:port 17006)))
;;;)

; Start the cogserver on port 17006
(start-cogserver "opencog-yue.conf")

; Open the database.
; Edit the below, setting the database name, user and password.
(sql-open "postgres:///yue_pairs?user=ubuntu&password=asdf")
