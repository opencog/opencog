(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog logger)
             (srfi srfi-1)
             (ice-9 readline)
             (ice-9 regex))

; ----------
; For debugging
(cog-logger-set-level! "debug")
(cog-logger-set-stdout! #t)

(define filename "cr.scm")

; ----------
(define (extract txt)
  "Extract the first term from the text."
  (define term-no-brackets (string-match "^[!'~_a-zA-Z0-9]+\\b" txt))
  (define term-with-brackets (string-match "^[[!'~a-zA-Z0-9 ]+\\b]*>*" txt))
  (cond ((not (equal? #f term-no-brackets))
         (match:substring term-no-brackets))
        ((not (equal? #f term-with-brackets))
         (match:substring term-with-brackets))))

(define (extract-term txt)
  "Identify and extract the term at the beginning of the text.
   Return the term extracted."
  (cond ; Since the quote (') could be used in two different
        ; terms, one is to indicate the literal occurrence of a
        ; word (e.g. 'played) and another is to indicate a group
        ; of words (e.g. 'Hanson Robotics'), so check if we
        ; got a quote for the first word
        ((string-prefix? "'" txt)
         (let ((sm (string-match "^'[a-zA-Z0-9 ]+\\b'" txt)))
           (if (equal? #f sm)
             ; Return the literal
             (extract txt)
             ; Return the group of words
             (match:substring sm))))
        ; Similarly angle brackets (< >) could be used in two
        ; different terms, one is to represent sentence
        ; anchors (e.g. "rose < I like"), and another is
        ; to represent unordered matching (e.g. <<like cats>>),
        ; so check if there are any angle brackets
        ((string-prefix? "<" txt)
         (let ((sm (string-match "^<<['~a-zA-Z0-9 ]+>>" txt)))
           (if (equal? #f sm)
             ; Return the sentence anchor <
             "<"
             ; Return the terms for unordered matching
             (match:substring sm))))
        ((string-prefix? ">" txt) ">")
        ; For handling choices -- square brackets (e.g. [like love])
        ((string-prefix? "[" txt)
         (let ((sm (string-match "^[['~a-zA-Z0-9 ]+]" txt)))
           (if (equal? #f sm)
             ; TODO: Do something else other then cog-logger-error?
             (cog-logger-error "Syntax error: ~a" txt)
             ; Return the choices
             (match:substring sm))))
        ; For the rest, like negation, concept, and lemma,
        ; just return the whole term
        (else (extract txt))))

(define* (identify-terms txt #:optional (lst '()))
  "Construct a list of terms by recursively extracting the terms
   one by one from the given text."
  (cog-logger-debug "Constructing term-list from: ~a" txt)
  (let* ((term (extract-term (string-trim-both txt)))
         (newtxt (string-trim (string-drop txt (string-length term))))
         (newlst (append lst (list term))))
    (cog-logger-debug "Term extracted: ~a" term)
    (if (< 0 (string-length newtxt))
      (identify-terms newtxt newlst)
      newlst)))

(define (subterm t n)
  "Remove the first and last n chars from the string t."
  (substring t n (- (string-length t) n)))

(define (join-terms func terms-str)
  "Identify the terms in a string, join them together with the name
   of a function."
  (string-append
    (fold (lambda (t s) (string-join (list s t) " "))
          (string-append "(" func)
          (interpret-terms (identify-terms terms-str)))
    ")"))

(define (interpret-terms terms)
  "Interpret the list of terms one by one and generate the actual
   function calls, in the form of a string."
  (map
    (lambda (t)
      (cond ; Group of words / phrase
            ((and (string-prefix? "'" t)
                  (not (equal? #f (string-match "^'[a-zA-Z0-9 ]+\\b'" t))))
             (string-append "(phrase \"" (subterm t 1) "\")"))
            ; Literal word
            ((string-prefix? "'" t)
             (string-append "(word \"" (substring t 1) "\")"))
            ; Unordered-matching
            ((string-prefix? "<<" t)
             (join-terms "unordered-matching" (subterm t 2)))
            ; Sentence anchor -- start of the sentence
            ((string-prefix? "<" t)
             "(anchor-start <)")
            ; Sentence anchor -- end of the sentence
            ((string-prefix? ">" t)
              "(anchor-end >)")
            ; Or-choices
            ((string-prefix? "[" t)
             (join-terms "or-choices" (subterm t 1)))
            ; TODO: Need negation-start, negation-end etc?
            ; Negation of a list of terms
            ((string-prefix? "![" t)
             (join-terms "negation" (substring t 2 (- (string-length t) 1))))
            ; Negation of a concept
            ((string-prefix? "!~" t)
             (join-terms "negation" (substring t 1)))
            ; Negation of a lemma
            ((string-prefix? "!" t)
             (string-append "(negation " t ")"))
            ; Concept
            ((string-prefix? "~" t)
             (string-append "(concept \"" (substring t 1) "\")"))
            ; Variable
            ; TODO: There are many types of variables...
            ((equal? "_" t)
             (string-append "(variable _)"))
            ; Part of speech (POS)
            ((not (equal? #f (string-match "[_a-zA-Z]+~" t)))
             (let ((ss (string-split t #\~)))
               (string-append "(" (cadr ss) " " (car ss) ")")))
            ; Lemma, the default case
            (else (string-append "(lemma \"" t "\")"))))
    terms))

(define (interpret-text txt)
  "Interpret the text in the pattern of the rule by firstly identifying
   the terms from the text and interpret them one by one later."
  (let* ((terms (identify-terms (string-trim-both txt)))
         (interp-terms (interpret-terms terms)))
    (cog-logger-debug "Total ~d terms were extracted: ~a" (length terms) terms)
    (cog-logger-debug "Term interpretation: ~a" interp-terms)
  interp-terms))

(define (write-to-file opat oact ipat iact)
  "Write the rule to a file, including both the original and interpreted form."
  (define outport (open-file filename "w"))
  (display "; (cr '" outport)
  (display opat outport)
  (display " '" outport)
  (display oact outport)
  (display "\n(chat-rule '" outport)
  (display ipat outport)
  (display " '" outport)
  (display iact outport)
  (display ")" outport)
  (close-output-port outport))

(define-public (cr pattern action)
  "Main function for creating a behavior rule.
   For verbal behavior, enclose the text in double quotes.

   Example Usage:
     (cr '(\"hi there\") '(\"hey\"))

   will create a rule -- if the input is \"hi there\",
   say \"hey\"."
  (define interp-pattern
    (append-map
      (lambda (p)
        (cog-logger-debug "Interpreting pattern: ~a" p)
        (cond
          ; The text input
          ((string? p)
           (interpret-text p))
          (else (cog-logger-debug "TODO: ~a" p))))
      pattern))

  (define interp-action
    (map
      (lambda (a)
        (cog-logger-debug "Interpreting action: ~a" a)
        (cond
          ((string? a)
           (string-append "(say " a ")"))
          (else (cog-logger-debug "TODO: ~a" a))))
      action))

  (write-to-file pattern
                 action
                 interp-pattern
                 interp-action)

  (cog-logger-info "Rule created for ~a ~a" pattern action)
  (cog-logger-info "Interpretation of the pattern: ~a" interp-pattern)
  (cog-logger-info "Interpretation of the action: ~a" interp-action)
)

(define (member-words w)
  "Extract the members of the concept."
  (let ((words (string-split w #\sp)))
    (if (= 1 (length words))
        (if (string-prefix? "~" (car words))
          (Concept (car words))
          (Word (car words)))
        (List (map-in-order Word words)))))

(define-public (chat-concept name . members)
  "Create named concepts with explicit membership lists.

   The first argument is the name of the concept, and the rest is the
   list of words and/or concepts that will be a member of the concept.

   Example Usage:
     (chat-concept \"eat\" \"ingest\" \"binge and purge\" \"~swallow\")

   will create a concept with the word \"ingest\", the phrase
   \"binge and purge\", and the concept \"swallow\" as its members."
  (let* ((c (Concept name))
         (ref-members (append-map (lambda (m) (list (Reference (member-words m) c)))
                                  members)))
    ref-members))
