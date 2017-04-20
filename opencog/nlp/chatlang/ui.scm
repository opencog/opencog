(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog logger)
             (ice-9 readline)
             (ice-9 regex))

; XXX TODO Remove me later
(cog-logger-set-level! "debug")
(cog-logger-set-level! "debug")
(cog-logger-set-stdout! #t)

(define (symbol-contains? sym str)
  "Check if a symbol contains a specific string."
  (not (equal? #f (string-contains (symbol->string sym) str))))

(define (get-first-term txt)
  "Get the first term from the text by detecting the word boundry"
  (match:substring (string-match "^['~a-zA-Z0-9]+\\b" txt)))

; XXX TODO Remove me later
; Note:
; In front of a word:
; Literal: '
; Concept: ~
; Proper Names: '
; Negation: !
; Or Choices: [
; Unordered Matching: <<
;
; After a word:
; POS: ~
; Proper Names: '
; Or Choices: ]
; Unordered Matching: >>
;
; Anywhere:
; Sentence Anchors: <
; Sentence Anchors: >
(define (extract-term txt)
  "Identify and extract the term at the beginning of the text."
  (cond ; Since the quote (') could be used in two different
        ; terms, one is to indicate the literal occurrence of a
        ; word (e.g. 'played) and another is to indicate a
        ; proper name (e.g. 'Hanson Robotics'), so check if we
        ; got a quote for the first word
        ((string-prefix? "'" txt)
         ; The regex here assumes that words for proper names are
         ; literally just words, no funny things like concepts
         ; negations, choices etc
         (let ((sm (string-match "^'[a-zA-Z0-9 ]+\\b'" txt)))
           ; Check whether it's a literal or proper name
           (if (equal? #f sm)
             ; Return the literal
             (get-first-term txt)
             ; Return the proper name
             (match:substring sm))))
        ; Similarly angle brackets (< >) could be used in two
        ; different terms, one is to represent sentence
        ; anchors (e.g. "rose < I like"), and another is
        ; to represent unordered matching (e.g. <<like cats>>),
        ; so check if there are any angle brackets
        ((string-prefix? "<" txt)
         ; The regex here assumes that words for unordered
         ; matching can only be words, lemmas, and concepts
         (let ((sm (string-match "^<<[a-zA-Z0-9 '~]+>>" txt)))
           (if (equal? #f sm)
             ; Return the sentence anchor <
             "<"
             ; Return the terms for unordered matching
             (match:substring sm))))
        ((string-prefix? ">" txt) ">")
        (else (get-first-term txt))))

(define (construct-lst txt lst)
  "Construct a list of terms by recursively extracting the
   terms one by one from the given text."
  (cog-logger-debug "Constructing term-list from: ~a" txt)
  (let* ((t (extract-term (string-trim-both txt)))
         (newtxt (string-trim (string-drop txt (string-length t))))
         (newlst (append lst (list t))))
    (cog-logger-debug "Term extracted: ~a" t)
    (if (< 0 (string-length newtxt))
      (construct-lst newtxt newlst)
      newlst)))

(define (interpret-text txt)
  "Interpret the text in the pattern of the rule by firstly
   extracting the terms from the text and interpret them
   one by one later."
  (let* ((terms (construct-lst (string-trim-both txt) '())))
    (cog-logger-debug "Total ~d terms were extracted: ~a" (length terms) terms))
)

(define-public (cr pattern action)
  "Main function for creating a behavior rule."
  (for-each
    (lambda (p)
      (display "p: ")
      (display p)
      (cond
        ; The text input
        ((string? p)
         (interpret-text p))
        ((symbol-contains? p ",")
         (display " (comma)"))
        ((symbol-contains? p "=")
         (display " (var)")))
      (newline))
    pattern)
  (for-each
    (lambda (a)
      (display "a: ")
      (display a)
      (cond
        ((string? a)
         (display " (output)"))
        ((symbol-contains? a ",")
         (display " (comma)"))
        ((symbol-contains? a "=")
         (display " (var)"))
        (else
         (display " (expression)")))
      (newline))
    action)
)

(define (s str) (quasiquote ,str))
;(define (b str) (quasiquote ,str)) ; placeholder for behaviour

(define (original-cr pattern action)
    (define strBuilder "")
    (define tmp "")
    (define wordtype "")
    (define x "")
    (define flag_or-choice 0)
    (define flag_proper-names 0)
    (define flag_unordered-matches 0)
    (define intCounter 0)
    (define intCounter2 0)
    (define pos_flag 1)
    (define (my-interpret var)
        ;;
        ;; Treat each word separately
        ;;
        (cond
            ; concept
            ((string-match "^[~][[:alnum:]]" var) (set! wordtype "concept"))
            ; word
            ((string-match "^['][[:alnum:]]" var) (set! wordtype "word"))
            ; main subject
            ((string-match "[[:alnum:]]~mainsubject" var)
             (begin
                 (set! wordtype "main-subj")
                 (if (string-match "~mainsubject" var)
                     (set! var (regexp-substitute #f (string-match "~mainsubject" var) 'pre "" 'post)))))
            ; main verb
            ((string-match "[[:alnum:]]~mainverb" var)
             (begin
                 (set! wordtype "main-verb")
                 (if (string-match "~mainverb" var)
                     (set! var (regexp-substitute #f (string-match "~mainverb" var) 'pre "" 'post)))))
            ; main object
            ((string-match "[[:alnum:]]~mainobject" var)
             (begin
                 (set! wordtype "main-obj")
                 (if (string-match "~mainobject" var)
                     (set! var (regexp-substitute #f (string-match "~mainobject" var) 'pre "" 'post)))))
            ; part of speech
            ((string-match "^[[:alpha:]]+~[[:alpha:]]+$" var)
             (begin
                 (set! wordtype "pos")
                 (set! var (regexp-substitute #f (string-match "[~]" var) 'pre "\" \"" 'post))))
            ; wildcard
            ((string-match "^([[:digit:]]+~[[:digit:]]+|[*])|[*]$" var)
             (begin
                 (set! wordtype "wildcard")
                 (if (string-match "^[[:digit:]]+[*]$" var)
                     (set! var (regexp-substitute #f (string-match "[*]" var) 'pre "" 'post)))))
            ; start-with <
            ((string-match "^[[:alnum:][:space:]]*[[:space:]]<[[:space:]][[:alnum:][:space:]]*$" var)
             (begin
                 (set! wordtype "start-with")
                 ; Discard everything before the '<'
                 (set! var (string-trim (regexp-substitute #f (string-match "^.*<" var) 'pre "" 'post)))
                 (set! var (regexp-substitute #f (string-match "[[:space:]]*" var) 'pre "\" \"" 'post))))
            ; XXX same as above?
            ; start-with <
            ((string-match ".*[[:space:]]*<[[:space:]]*.*" var)
             (begin
                 ;(set! var (string-trim (regexp-substitute #f (string-match "<" var) 'pre "" 'post))) ; discard everything before the '<'
                 ;(set! var (regexp-substitute #f (string-match "[[:space:]]*" var) 'pre "" 'post))
                 (set! wordtype "start-with")))
            (else (set! wordtype "lemma")))

        (string-append "(" wordtype " \"" var "\"" ") " )

        ;(for-each (lambda (x) (set! result (string-append result x))))
        ;result
    )

;;
;; Process groups of words
;;
; XXX syntax error, to be removed?
#!
(define (my-interpret-group x name leftMatch rightMatch flag)
    ((or (string-match leftMatch x) (equal? flag 1))
        (if (string-match leftMatch tmp) (set! tmp (regexp-substitute #f (string-match leftMatch tmp) 'pre "" 'post)))
            (if (string-match rightMatch tmp) (set! tmp (regexp-substitute #f (string-match rightMatch tmp) 'pre "" 'post)))
                ;(display  intCounter2)
                (if (equal? intCounter2 0)
                    (begin
                        (set! strBuilder (string-append strBuilder "(" name " "))
                            (if (equal? tmp "") (display "") (set! strBuilder (string-append strBuilder "\""  tmp "\" ")))
                            (set! intCounter2 1))
                    (if (not (equal? tmp ""))  (set! strBuilder (string-append strBuilder  "\"" tmp "\" "))))
            (if (string-match rightMatch x)
                (begin
                    (set! flag 0)
                    (set! strBuilder (string-append strBuilder ") ")))
                (set! flag 1)
                ;;(set! intCounter 0)
            )))
!#

    (set! pattern (string-trim pattern))
    (if (string-match "^#" pattern)
        (display "exit\n")
        ; exit if there is a comment # at start of string
        (begin
            (set! strBuilder (string-append strBuilder "(chat-rule\n    '("))
            ;; sentense level processing

; XXX to be removed?
#!
(if (string-match "^.*[[:space:]]<[[:space:]].*$" pattern)
    (begin ; start-with <
        (set! tmp pattern)
        (set! tmp (string-trim (regexp-substitute #f (string-match "^.*<" tmp) 'pre "" 'post))) ; discard everything before the '<'
        (set! tmp (string-trim (regexp-substitute #f (string-match "[[:space:]]*" (string-trim tmp)) 'pre "" 'post)))
        (set! strBuilder (string-append strBuilder "(start-with \"" tmp "\"" ") " ))
        ;(display (string-append "\n\n" strBuilder "\n\n"))
        ;(set! strBuilder (string-append strBuilder (my-interpret x))
    ))
!#

            ; (set! strBuilder (string-append strBuilder (string-concatenate (map my-interpret (string-split pattern #\space)))))

            ; (for-each (lambda (x) (set! strBuilder (string-append strBuilder (string-concatenate (map my-interpret x)))
            (for-each
                (lambda (x)
                    (set! x (string-trim x))
                    (set! tmp (string-trim (string-filter (lambda (c) (not (or (char=? #\[ c) (char=? #\] c)))) x)))
                    (cond
                        ;; or-choices
                        ((or (string-match "\\[" x) (equal? flag_or-choice 1))
                         (if (equal? intCounter 0)
                            (begin
                               (set! strBuilder (string-append strBuilder "(or-choices "))
                               (if (not (equal? tmp ""))
                                   (set! strBuilder (string-append strBuilder "\""  tmp "\" ")))
                               (set! intCounter 1))
                            (if (not (equal? tmp ""))
                                (set! strBuilder (string-append strBuilder  "\"" tmp "\" "))))
                         (if (string-match "\\]" x)
                             (begin
                                 (set! flag_or-choice 0)
                                 (set! strBuilder (string-append strBuilder ") ")))
                             (set! flag_or-choice 1)))

                        ;; proper-names
                        ((or (string-match "^''" x) (equal? flag_proper-names 1))
                         ;(set! tmp (string-trim (string-filter (lambda (c) (not  (char=? #\' c))) x)))
                         (if (string-match "''" tmp)
                             (set! tmp (regexp-substitute #f (string-match "''" tmp) 'pre "" 'post)))
                         (if (equal? intCounter 0)
                            (begin
                                (set! strBuilder (string-append strBuilder "(proper-names "))
                                (if (equal? tmp "")
                                    ; XXX
                                    (display "")
                                    (set! strBuilder (string-append strBuilder "\""  tmp "\" ")))
                                (set! intCounter 1))
                            (if (not (equal? tmp ""))
                                (set! strBuilder (string-append strBuilder  "\"" tmp "\" "))))
                         (if (string-match "['']$" x)
                            (begin
                                (set! flag_proper-names 0)
                                (set! strBuilder (string-append strBuilder ") ")))
                            (set! flag_proper-names 1)
                            ;;(set! intCounter 0)
                         ))

                        ; (my-interpret-group x "proper-names" "''" "''" flag_proper-names) ;leftMatch rightMatch flag)

                        ;; unordered matches
                        ((or (string-match "<<" x) (equal? flag_unordered-matches 1))
                         (if (string-match "<<" tmp)
                             (set! tmp (regexp-substitute #f (string-match "<<" tmp) 'pre "" 'post)))
                         (if (string-match ">>" tmp)
                             (set! tmp (regexp-substitute #f (string-match ">>" tmp) 'pre "" 'post)))
                         ;(display  intCounter2)
                         (if (equal? intCounter2 0)
                            (begin
                                (set! strBuilder (string-append strBuilder "(unordered-matches "))
                                (if (equal? tmp "")
                                    ; XXX
                                    (display "")
                                    (set! strBuilder (string-append strBuilder "\""  tmp "\" ")))
                                (set! intCounter2 1))
                            (if (not (equal? tmp ""))
                                (set! strBuilder (string-append strBuilder  "\"" tmp "\" "))))
                         (if (string-match ">>" x)
                             (begin
                                 (set! flag_unordered-matches 0)
                                 (set! strBuilder (string-append strBuilder ") ")))
                             (set! flag_unordered-matches 1)
                             ;;(set! intCounter 0)
                         ))

                        ;; Ignore everything after # because it's a comment
                        ;( (string-match "#" x)
                            ;; how to break out of a for-each?
                        ;)

                        ;; single word processing
                        (else
                            (if (equal? x "")
                                ; XXX
                                (display "")
                                (set! strBuilder (string-append strBuilder (my-interpret x)))))
                    )
                )
            (string-split pattern #\space))

; XXX to be removed?
; startwith alternate code
;(if (string-match "^.*[[:space:]]<[[:space:]].*$" pattern) (begin ; start-with <
;        (set! tmp pattern)
;        (set! tmp (string-trim (regexp-substitute #f (string-match "^.*<" tmp) 'pre "" 'post))) ; discard everything before the '<'
;        (set! tmp (string-trim (regexp-substitute #f (string-match "[[:space:]]*" (string-trim tmp)) 'pre "" 'post)))
;        (set! strBuilder (string-append strBuilder "(start-with \"" tmp "\"" ") " ))
;        ;(display (string-append "\n\n" strBuilder "\n\n"))
;        ;(set! strBuilder (string-append strBuilder (my-interpret x))
;))

            (set! strBuilder (string-append strBuilder ")\n"))
            (set! strBuilder (string-append strBuilder "    '(say \"" action "\"))" ))
            (display strBuilder)
            (newline)
            ;(eval-string strBuilder)
        )
    )
)
