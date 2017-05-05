(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog logger)
             (srfi srfi-1))

; ----------
; For debugging
(cog-logger-set-level! "debug")
(cog-logger-set-stdout! #t)

; ----------
(define (extract TXT)
  "Extract the first term from the text."
  (define term-no-brackets (string-match "^[!'~_a-zA-Z0-9]+\\b" TXT))
  (define term-with-brackets (string-match "^[[!'~a-zA-Z0-9 ]+\\b]*>*" TXT))
  (cond ((not (equal? #f term-no-brackets))
         (match:substring term-no-brackets))
        ((not (equal? #f term-with-brackets))
         (match:substring term-with-brackets))))

(define (extract-term TXT)
  "Identify and extract the term at the beginning of the text."
  (cond ; Since the quote (') could be used in two different
        ; terms, one is to indicate the occurrence of a literal
        ; word (e.g. 'played) and another is to indicate the
        ; occurrence of a group of words (e.g. 'Hanson Robotics'),
        ; so check if we got a quote for the first word
        ((string-prefix? "'" TXT)
         (let ((sm (string-match "^'[a-zA-Z0-9 ]+\\b'" TXT)))
           (if (equal? #f sm)
               ; Return the literal
               (extract TXT)
               ; Return the group of words
               (match:substring sm))))
        ; Similarly angle brackets (< >) could be used in two
        ; different terms, one is to represent sentence
        ; anchors (e.g. "rose < I like"), and another is
        ; to represent unordered matching (e.g. <<like cats>>),
        ; so check if there are any angle brackets
        ((string-prefix? "<" TXT)
         (let ((sm (string-match "^<<['~a-zA-Z0-9 ]+>>" TXT)))
           (if (equal? #f sm)
               ; Return the sentence anchor <
               "<"
               ; Return the terms for unordered matching
               (match:substring sm))))
        ((string-prefix? ">" TXT) ">")
        ; For handling choices -- square brackets (e.g. [like love])
        ((string-prefix? "[" TXT)
         (let ((sm (string-match "^[['~a-zA-Z0-9 ]+]" TXT)))
           (if (equal? #f sm)
               ; TODO: Do something else other then cog-logger-error?
               (cog-logger-error "Syntax error: ~a" TXT)
               ; Return the choices
               (match:substring sm))))
        ; For the rest, like negation, concept, and lemma,
        ; just return the whole term
        (else (extract TXT))))

(define* (identify-terms TXT #:optional (LST '()))
  "Construct a list of terms by recursively extracting the terms
   one by one from the given text."
  (cog-logger-debug "Constructing term-list from: ~a" TXT)

  (let* ((term (extract-term (string-trim-both TXT)))
         (newtxt (string-trim (string-drop TXT (string-length term))))
         (newlst (append LST (list term))))

    (cog-logger-debug "Term extracted: ~a" term)

    (if (< 0 (string-length newtxt))
        (identify-terms newtxt newlst)
        newlst)))

(define (subterm STR NUM)
  "Remove the first and last n chars from the string."
  (substring STR NUM (- (string-length STR) NUM)))

(define (interpret-terms TERMS)
  "Interpret the list of terms one by one and generate the actual
   function calls, in the form of a string."
  (map (lambda (t)
    (cond ; Group of words / phrase
          ((and (string-prefix? "'" t)
                (not (equal? #f (string-match "^'[a-zA-Z0-9 ]+\\b'" t))))
           (cons 'phrase (subterm t 1)))
          ; Literal word
          ((string-prefix? "'" t)
           (cons 'word (substring t 1)))
          ; Unordered-matching
          ((string-prefix? "<<" t)
           (cons 'unordered-matching
                 (interpret-terms (identify-terms (subterm t 2)))))
          ; Sentence anchor -- start of the sentence
          ((string-prefix? "<" t)
           (cons 'anchor-start "<"))
          ; Sentence anchor -- end of the sentence
          ((string-prefix? ">" t)
           (cons 'anchor-end ">"))
          ; Choices
          ((string-prefix? "[" t)
           (cons 'choices
                 (interpret-terms (identify-terms (subterm t 1)))))
          ; TODO: Need negation-start, negation-end etc?
          ; Negation of a list of terms
          ((string-prefix? "![" t)
           (cons 'negation
                 (interpret-terms (identify-terms
                   (substring t 2 (- (string-length t) 1))))))
          ; Negation of a concept
          ((string-prefix? "!~" t)
           (cons 'negation (substring t 1)))
          ; Negation of a lemma
          ((string-prefix? "!" t)
           (cons 'negation t))
          ; Concept
          ((string-prefix? "~" t)
           (cons 'concept (substring t 1)))
          ; Variable
          ; TODO: There are many types of variables...
          ; ((equal? "_" t)
          ;  (string-append "(variable _)"))
          ; Part of speech (POS)
          ; ((not (equal? #f (string-match "[_a-zA-Z]+~" t)))
          ;  (let ((ss (string-split t #\~)))
          ;    (string-append "(" (cadr ss) " " (car ss) ")")))
          ; Lemma, the default case
          (else (cons 'lemma t))))
    TERMS))

(define (interpret-text TXT)
  "Process the text in the pattern of the rule by firstly identifying
   the terms from the text and interpret them one by one later."
  (define terms (identify-terms (string-trim-both TXT)))
  (define interp-terms (interpret-terms terms))

  (cog-logger-debug "Total ~d terms were extracted: ~a" (length terms) terms)
  (cog-logger-debug "Term interpretation: ~a" interp-terms)

  interp-terms)

(define-public (cr PATTERN ACTION)
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
        (cond ; The text input
              ((string? p)
               (interpret-text p))
              (else (cog-logger-debug "TODO: ~a" p))))
      PATTERN))

  (define interp-action
    (append-map
      (lambda (a)
        (cog-logger-debug "Interpreting action: ~a" a)
        (cond ; The text output
              ((string? a)
               (cons 'say a))
              (else (cog-logger-debug "TODO: ~a" a))))
      ACTION))

  (cog-logger-info "Interpretation of the pattern: ~a" interp-pattern)
  (cog-logger-info "Interpretation of the action: ~a" interp-action)

  ; Now create the actual psi-rule
  (chat-rule interp-pattern interp-action))

(define (member-words STR)
  "Convert the member in the form of a string into an atom, which can
   either be a WordNode, a ConceptNode, or a ListLink of WordNodes."
  (let ((words (string-split STR #\sp)))
    (if (= 1 (length words))
        (if (string-prefix? "~" (car words))
            (Concept (substring (car words) 1))
            (Word (car words)))
        (List (map-in-order Word words)))))

(define-public (create-concept NAME . MEMBERS)
  "Create named concepts with explicit membership lists.

   The first argument is the name of the concept, and the rest is the
   list of words and/or concepts that will be considered as the members
   of the concept.

   Example Usage:
     (create-concept \"eat\" \"ingest\" \"binge and purge\" \"~swallow\")

   will create a concept with the word \"ingest\", the phrase
   \"binge and purge\", and the concept \"swallow\" as its members."
  (append-map (lambda (m) (list (Reference (member-words m) (Concept NAME))))
              MEMBERS))
