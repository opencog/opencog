(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog logger)
             (srfi srfi-1)
             (ice-9 readline)
             (ice-9 regex))

; XXX TODO Remove me later
(cog-logger-set-level! "debug")
(cog-logger-set-stdout! #t)

; The regex here assumes that words for proper names are
; literally just words, no funny things like concepts
; negations, choices etc
(define regex-proper-names "^'[a-zA-Z0-9 ]+\\b'")

; The regex here assumes that words for unordered
; matching can only be words, lemmas, and concepts
(define regex-unordered "^<<[a-zA-Z0-9 '~]+>>")

; The regex here assumes that the choices can only be
; words, lemmas, and concepts
(define regex-choices "^[[a-zA-Z0-9 '~]+]")

; The regex to extract a negation term e.g. !greet or !~food etc
(define regex-negterm "^![~'a-zA-Z0-9]+\\b")

; The regex to extract the negation a list of terms
; e.g. ![hi hey ~others]
(define regex-neglst "^![[<'~a-zA-Z0-9 ]+\\b]*>*")

(define (symbol-contains? sym str)
  "Check if a symbol contains a specific string."
  (not (equal? #f (string-contains (symbol->string sym) str))))

(define (get-first-term txt)
  "Get the first term from the text."
  (define term-no-brackets (string-match "^[!'~_a-zA-Z0-9]+\\b" txt))
  (define term-with-brackets (string-match "^[[!'~a-zA-Z0-9 ]+\\b]*>*" txt))
  (cond ((not (equal? #f term-no-brackets))
         (match:substring term-no-brackets))
        ((not (equal? #f term-with-brackets))
         (match:substring term-with-brackets))))

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
; Variable: _

(define (extract txt)
  "Identify and extract the term at the beginning of the text."
  (cond ; Since the quote (') could be used in two different
        ; terms, one is to indicate the literal occurrence of a
        ; word (e.g. 'played) and another is to indicate a
        ; proper name (e.g. 'Hanson Robotics'), so check if we
        ; got a quote for the first word
        ((string-prefix? "'" txt)
         (let ((sm (string-match regex-proper-names txt)))
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
         (let ((sm (string-match regex-unordered txt)))
           (if (equal? #f sm)
             ; Return the sentence anchor <
             "<"
             ; Return the terms for unordered matching
             (match:substring sm))))
        ((string-prefix? ">" txt) ">")
        ; For handling choices -- square brackets (e.g. [like love])
        ((string-prefix? "[" txt)
         (let ((sm (string-match regex-choices txt)))
           (if (equal? #f sm)
             ; TODO: Do something else other then cog-logger-error
             (cog-logger-error "Syntax error: ~a" txt)
             ; Return the choices
             (match:substring sm))))
        ; For the rest, like negation, concept, and lemma,
        ; just return the whole term
        (else (get-first-term txt))))

(define* (extract-terms txt #:optional (lst '()))
  "Construct a list of terms by recursively extracting the
   terms one by one from the given text."
  (cog-logger-debug "Constructing term-list from: ~a" txt)
  (let* ((term (extract (string-trim-both txt)))
         (newtxt (string-trim (string-drop txt (string-length term))))
         (newlst (append lst (list term))))
    (cog-logger-debug "Term extracted: ~a" term)
    (if (< 0 (string-length newtxt))
      (extract-terms newtxt newlst)
      newlst)))

(define (rearrange-terms terms)
  "Check if the terms have any sentence anchors and rearrange them in
   the intended orders."
  (define (check-anchor-start terms)
    (define starting-terms (member "<" terms))
    ; Take all the terms after the sentence anchor "<" to the front
    ; of the list, if any, but not including the "<" in the new list
    (if (equal? #f starting-terms)
      ; TODO: Add a zero-to-many-GlobNode in front of it
      terms
      ; TODO: Should add a zero-to-many-GlobNode in between the
      ; starting terms and the rest?
      (append (cdr starting-terms)
              (take-while (lambda (x) (not (equal? "<" x))) terms))))
  (define (check-anchor-end terms)
    (if (equal? #f (member ">" terms))
      ; TODO: Add a zero-to-many-GlobNode at the end
      terms
      (remove (lambda (x) (equal? ">" x)) terms)))
  ; Should check for "<" before ">" in case both of them happen
  ; to exist in the pattern
  (check-anchor-end (check-anchor-start terms)))

(define (subterm t n)
  "Remove the first and last n chars from the string t."
  (substring t n (- (string-length t) n)))

(define (interpret-terms terms)
  "Interpret the terms one by one, and generate the actual function
   calls."
  (map
    (lambda (t)
      (cond ((and (string-prefix? "'" t)
                  (not (equal? #f (string-match regex-proper-names t))))
             (cons "proper-names" (string-tokenize (subterm t 1))))
            ((string-prefix? "'" t)
             (cons "word" (list (substring t 1))))
            ((string-prefix? "<<" t)
             (cons "unordered-matching"
               (interpret-terms (extract-terms (subterm t 2)))))
            ((string-prefix? "<" t)
             (cons "anchor-start" "<"))
            ((string-prefix? ">" t)
             (cons "anchor-end" ">"))
            ((string-prefix? "[" t)
             (cons "or-choices"
               (interpret-terms (extract-terms (subterm t 1)))))
            ; TODO: negation-start, negation-end etc?
            ((and (string-prefix? "!" t)
                  (not (equal? #f (string-match regex-negterm t))))
             (cons "negation" (list (substring
               (match:substring (string-match regex-negterm t)) 1))))
            ((and (string-prefix? "!" t)
                  (not (equal? #f (string-match regex-neglst t))))
             (cons "negation" (interpret-terms (extract-terms
               (substring (match:substring (string-match regex-neglst t)) 1)))))
            ((string-prefix? "~" t)
             (cons "concept" (list (substring t 1))))
            ((equal? "_" t)
             (cons "variable" "_"))
            ((not (equal? #f (string-match "[_a-zA-Z]+~" t)))
             (let ((ss (string-split t #\~)))
               (cons (cadr ss) (car ss))))
            ; Consider as lemma by default
            (else (cons "lemma" (list t)))))
    terms))

(define (interpret-text txt)
  "Interpret the text in the pattern of the rule by firstly
   extracting the terms from the text and interpret them
   one by one later."
  (let* ((terms (extract-terms (string-trim-both txt)))
         (interp-terms (interpret-terms terms)))
;         (sorted-terms (rearrange-terms terms))
;         (interp-terms (interpret-terms sorted-terms)))
    (cog-logger-debug "Total ~d terms were extracted: ~a" (length terms) terms)
;    (cog-logger-debug "Total ~d terms remained after rearranging: ~a"
;      (length sorted-terms) sorted-terms)
    (cog-logger-debug "Term interpretation: ~a" interp-terms)))

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
