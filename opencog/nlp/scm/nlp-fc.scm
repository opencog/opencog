(define (nlp-files-fc sentence-file rules-file)
  ;rulebase-func r2l-rules
  (load-scm-from-file rules-file)
  (cog-fc
  (SetLink (load-sentences sentence-file))
  r2l-rules
  )
)

;(define (load-rules-from-file fl)
;)

(define (load-sentences sfile)
  (map-to-relex-parse-atoms (map-sent-nodes-to-parse (nlp-parse-from-file-mod sfile)))
)
;(parse-get-relations (car(sentence-get-parses (car(nlp-parse "how are you doing")))))
;(for-each (lambda (x) (display x)) (list "hello" "there"))
(define (nlp-parse-from-file-mod filepath)
  (define sent-node-list '())
    (let*
        ((cmd-string (string-join (list "cat " filepath) ""))
        (port (open-input-pipe cmd-string))
        (line (get-line port))
        )
        (while (not (eof-object? line))
            (if (or (= (string-length line) 0)
                    (char=? #\; (string-ref (string-trim line) 0))
                )
                (set! line (get-line port))
                (if (string=? "END." line)  ; continuing the tradition of RelEx
                    (break)
                    (begin
                        (catch #t
                            (lambda ()
                                (set! sent-node-list (append sent-node-list (nlp-parse line)))
                            )
                            (lambda (key . parameters)
                                (begin
                                    (display "*** Unable to parse: \"")
                                    (display line)
                                    (display "\"\n")
                                )
                            )
                        )
                        (set! line (get-line port))
                    )
                )
            )
        )
        sent-node-list
        ;(close-pipe port)
    )
)

(define (map-sent-nodes-to-parse sent-node-list)
  (define mylist '())
  (for-each (lambda (x)(set! mylist (append mylist (sentence-get-parses x))))sent-node-list)
  mylist
)

(define (map-to-relex-parse-atoms parse-node-list)
  (define mylist '())
  (for-each (lambda (x)(set! mylist (append mylist (parse-get-relations x))))parse-node-list)
  mylist
)
