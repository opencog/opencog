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

; localhost, port number 18001  (shell-port is set in config.scm)
(spawn-server (make-tcp-server-socket  #:port shell-port))

(repl-default-prompt-set! shell-prompt)
; --------------------------------------------------------------------
