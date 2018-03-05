;; This contains functions (system and custom) available for use
;; in any GHOST rules. Basically things that you can called with this syntax:
;;   ^function_name(arguments)
;; For example:
;;   u: (hi robot) ^reuse(GREETINGS)

; TODO: Create an interface to avoid having too many custom functions with generic names?

(define-public (reuse LABEL)
"
  The function for reusing the output of another rule labeled with LABEL.
"
  (define rule (cog-chase-link 'ListLink 'ImplicationLink
                 (Concept (string-append psi-prefix-str (cog-name LABEL)))))
  (if (null? rule)
      (cog-logger-error ghost "Failed to find the GHOST rule \"~a\"" (cog-name LABEL))
      ; TODO: Should avoid needing to call "cog-evaluate!" manually
      (let ((action (psi-get-action (car rule))))
           (cog-evaluate! action)
           action)))
