#! /usr/bin/guile -s
!#

(define si-file (open-file (cadr (command-line)) "r"))
(define output-file (open-file (caddr (command-line)) "w"))
(define s-files (cdddr (command-line)))

(load "/home/opencog/.guile")
(use-modules (system repl server))
(use-modules (ice-9 regex))
(use-modules (rnrs bytevectors))
(use-modules (srfi srfi-1))
(use-modules (rnrs io ports))
(use-modules (opencog)
            (opencog nlp)
            (opencog nlp relex2logic)
            (opencog nlp fuzzy)
            (opencog nlp sureal)
            (opencog nlp chatbot)
            (opencog nlp aiml)
            (opencog exec)
            (opencog python)
            (opencog openpsi))

(define command "")

(define (exec-command) 
    (primitive-eval (read (open-input-string command)))
)

; Read and nlp-parse sentences
(let (
    (line "")
    (fin "")
    (error #f)
) (begin
    (for-each (lambda (s-file-name) (begin
        (display "Reading sentences from ")
        (display s-file-name) (newline)
        (set! fin (open-file s-file-name "r"))
        (while (not (eof-object? line)) (begin
            (set! line (get-line fin))
            (if (not (eof-object? line)) (begin
                (catch #t (lambda ()
                    (nlp-parse line)
                    ;(display line) (newline)
                ) (lambda (key . parameters)
                    (set! error #t)
                ))
            ))
        ))
    )) s-files)
))

; Call SuReal and record its output
(let (
    (sentence "")
    (sureal-result "")
) (begin
    (display "Processing SuReal queries from ")
    (display (cadr (command-line)))
    (newline)
    (while (not (eof-object? sentence)) (begin
        (set! sentence (get-line si-file))
        (display sentence output-file)
        (display "\n" output-file)
        (if (not (eof-object? sentence)) (begin
            (set! command (get-line si-file))
            (catch #t (lambda () (begin
                (set! sureal-result 
                    (join-thread 
                        (call-with-new-thread 
                            exec-command
                        )
                        (+ (current-time) (* 10 60))
                        "TIMEOUT"
                    )
                )
                (if (null? sureal-result) (begin
                    (display "EMPTY\n" output-file)
                ) (begin
                    (display sureal-result output-file)
                    (display "\n" output-file)
                ))
            )) (lambda (key . parameters) (begin
                (display key)
                (display " ")
                (display parameters)
                (newline)
                (display "ERROR\n" output-file)
            )))
        ))
        (fsync output-file)
    ))
))


