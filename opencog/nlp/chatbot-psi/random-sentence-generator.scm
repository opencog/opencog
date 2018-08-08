; XXX Temp quick hacks for the upcoming demos
; The design is horrible to call the generator like this

; Example usage of the generator directly from the command line
; ruby marky_markov speak -d ~/.marky_markov_dictionary -c 1 -p "reality"

; Files and dictionaries being used are available here:
; https://github.com/opencog/test-datasets/releases/download/current/markov_modeling.tar.gz

;------------------------------------------------------------------------------

(use-modules (ice-9 popen) (rnrs io ports))
(load "states.scm")

;------------------------------------------------------------------------------

(define has-markov-setup #f)
(define markov-bin "")
(define markov-dict "")
(define pkd-relevant-words '())
(define blog-relevant-words '())
(define kurzweil-relevant-words '())

(define-public (markov-setup bin-dir dict-dir)
    (define (read-words word-list)
        (let* ((cmd (string-append "cat " dict-dir "/" word-list))
               (port (open-input-pipe cmd))
               (line (get-line port))
               (strong-words-start #f)
               (results '()))
            (while (not (eof-object? line))
                ; Ignore empty lines
                (if (= (string-length (string-trim line)) 0)
                    (set! line (get-line port))
                    (begin
                        ; There are WEAK and STRONG words, only gets STRONG ones
                        (if strong-words-start
                            (set! results (append results (list (string-trim line))))
                            (if (equal? "STRONG" (string-trim line))
                                (set! strong-words-start #t))
                        )
                        (set! line (get-line port))
                    )
                )
            )
            (close-pipe port)
            results
        )
    )

    (set! markov-bin (string-append bin-dir "/marky_markov"))
    (set! markov-dict dict-dir)
    (set! pkd-relevant-words (read-words "PKD_relevant_words.txt"))
    (set! blog-relevant-words (read-words "blog_relevant_words.txt"))
    (set! kurzweil-relevant-words (read-words "Kurzweil_relevant_words.txt"))
    (State random-pkd-sentence-generator default-state)
    (State random-blogs-sentence-generator default-state)
    (State random-kurzweil-sentence-generator default-state)
    (set! has-markov-setup #t)
)

(define (call-random-sentence-generator dict-node)
    (begin-thread
        (define dict (cog-name dict-node))
        (define random-sentence-generator
            (cond ((equal? dict "pkd") random-pkd-sentence-generator)
                  ((equal? dict "blogs") random-blogs-sentence-generator)
                  ((equal? dict "kurzweil") random-kurzweil-sentence-generator)))
        (define random-sentence-generated
            (cond ((equal? dict "pkd") random-pkd-sentence-generated)
                  ((equal? dict "blogs") random-blogs-sentence-generated)
                  ((equal? dict "kurzweil") random-kurzweil-sentence-generated)))

        (State random-sentence-generator process-started)

        (let* ((cmd (string-append "ruby " markov-bin " speak -d " markov-dict
                    "/" dict " -c 1 -p \"" rsg-input "\""))
               (port (open-input-pipe cmd))
               (line (get-line port)))

            (if (eof-object? line)
                (State random-sentence-generated no-result)
                (State random-sentence-generated
                    (List (map Word (string-split line #\ ))))
            )

            (State random-sentence-generator process-finished)

            (close-pipe port)
        )
    )

    ; Return for the GroundedSchemaNode
    (Set)
)
