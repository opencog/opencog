(use-modules (opencog logger))

; --------------------------------------------------------------
; Configure eva logger
(define ebl (cog-new-logger))
(cog-logger-set-component! ebl "Eva")
(cog-logger-set-level! ebl "info")
(cog-logger-set-stdout! ebl #f)

(define-public (eva-get-logger)
"
  eva-get-logger

  Returns the looger for eva-model module. This is a logger to be used by all
  eva-* modules.
"
  ebl
)

; --------------------------------------------------------------
(define-public (print-msg node)
"
  print-msg NODE

  NODE's name is the message logged when the logger for this module is on
  DEBUG level.
"
  (cog-logger-debug ebl "~a" (cog-name node))
  (stv 1 1))
