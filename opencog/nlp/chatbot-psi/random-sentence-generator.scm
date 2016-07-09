; XXX Temp quick hacks for the upcoming demos
; The design is horrible to call the generator like this

; Example usage of the generator directly from the command line
; ruby marky_markov speak -d ../../markov_modeling/blogs -c 1 -p "It's hot today"

;------------------------------------------------------------------------------

(use-modules (ice-9 popen) (rnrs io ports))

;------------------------------------------------------------------------------

(define markov-bin "")
(define markov-dict "")

(define-public (markov_setup bin-dir dict-dir)
    (set! markov-bin (string-append bin-dir "/marky_markov"))
    (set! markov-dict dict-dir)
)

(define (call-random-sentence-generator dict)
    (if (not (or (equal? markov-bin "") (equal? markov-dict "")))
        (let* ((input-text (cog-name (get-input-text-node)))
               (cmd (string-append "ruby " markov-bin " speak -d " markov-dict "/" dict " -c 1 -p \"" input-text "\""))
               (port (open-input-pipe cmd))
               (line (get-line port)))
            ; Create a list of WordNodes wrapped in a ListLink
            ; TODO
            (List (map Word (string-split input-text #\ )))
        )
    )
)
