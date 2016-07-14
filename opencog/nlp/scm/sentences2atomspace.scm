(use-modules 
    (ice-9 popen)    ; needed for open-pipe, close-pipe
    (rnrs io ports)  ; needed for get-line
)

(load-from-path "/opencog/opencog/nlp/chatbot/chat-utils.scm")
(let*
    (
        (cmd-string (string-join (list "cat " "sentences.txt") ""))
        (port (open-input-pipe cmd-string))
        (line (get-line port))
    )
    (while (not (eof-object? line))
        (begin
            (clear)
            (catch #t
                (lambda ()
                    (nlp-parse line)
                )
                (lambda (key . parameters)
                    (begin
                        (display "Parse error. Unable to parse sentence: \"" (current-error-port))
                        (display line (current-error-port))
                        (display "\"\n" (current-error-port))
                    )
                )
            )
            (cog-prt-atomspace)
            (set! line (get-line port))
        )
    )
    (close-pipe port)
)
(display (count-all))
(newline)
,q
