;
; repl-shell.scm
;
; Start the "high-level" scheme shell.
;
; Copyright (C) 2013 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------

(use-modules (system repl server))
(use-modules (system repl common))

; localhost, port number 18001
(spawn-server (make-tcp-server-socket  #:port 18001))

(repl-default-prompt-set! "opencog-scheme>")
; --------------------------------------------------------------------
