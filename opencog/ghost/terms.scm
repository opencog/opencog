;; Assorted functions for translating individual terms into Atomese fragments.
;; A term is like a "feature" that can be used when creating a GHOST rule.

; ----------
(define (word STR)
"
  Occurrence of a word, a word that should be matched literally.
"
  (let* ((str-dc (string-downcase STR))
         ; Special handling for time that's written as a single word
         ; e.g. 2pm, 10am etc
         ; For the input, regardless of the format, e.g. "2am", "2 am",
         ; "2 a.m." or "2a.m.", will all get splitted into two words
         (is-time? (string-match "[0-9]{1,2}[ ]*[ap][.]*m[.]*" str-dc))
         (is-two-digits?
           (and is-time? (char-numeric? (string-ref str-dc 1))))
         (time-1pt
           (if is-two-digits?
             (substring str-dc 0 2)
             (substring str-dc 0 1)))
         (time-2pt
           (if is-two-digits?
             (substring str-dc 2)
             (substring str-dc 1))))
    (list (list) (list)
      (if is-time?
        (list (WordNode time-1pt) (WordNode time-2pt))
        (list (WordNode str-dc))))))

; ----------
(define (word-apos STR)
"
  Occurrence of a word with an apostrophe, e.g. I'm, it's etc.
  Should be matched literally.
"
  (let* (; This turns ’ into ' just to treat them as the same thing
         (nstr (regexp-substitute/global #f "’" STR 'pre "'" 'post))
         (w (WordNode (string-downcase nstr))))
    (list (list) (list) (list w))))

; ----------
(define (lemma STR)
"
  Lemma occurrence, aka canonical form of a term.
  This is the default for word mentions in the rule pattern.
"
  (let* ((str-dc (string-downcase STR))
         (var (Variable (gen-var str-dc #t)))
         (l (WordNode (get-lemma str-dc)))
         (v (list (TypedVariable var (Type "WordNode"))))
         (c (list ; In some rare situation, particularly if the input
                  ; sentence is not grammatical, RelEx may not lemmatize a
                  ; word because of the ambiguity
                  ; So just to be sure "l" is the stem of "var",
                  ; a GroundedPredicateNode is used instead of putting
                  ; "(LemmaLink v2 l)" in the context
                  (Evaluation (GroundedPredicate "scm: ghost-lemma?")
                              (List var l)))))
    ; Special handling for the word "do"
    ; Given just one word, without any other clues, it seems impossible to
    ; determine whether the word "does" from the input means "perform" (do)
    ; or "a female deer" (doe), so make sure, in this case, that the RelEx
    ; outputs will be considered when "do" is in the pattern of a rule
    (if (string-ci=? STR "do")
      (let ((wi-var (Variable (gen-var str-dc #f))))
        (set! v (append v (list
          (TypedVariable wi-var (Type "WordInstanceNode")))))
        (set! c (list
          (ReferenceLink wi-var var)
          (WordInstanceLink wi-var (Variable "$P"))
          (LemmaLink wi-var (Word "do"))))))
    (list v c (list var))))

(define-public (ghost-lemma? GRD LEMMA)
"
  Check if LEMMA is the stem of GRD.
"
  (cog-logger-debug ghost-logger
    "In ghost-lemma? LEMMA: ~aGRD: ~a" LEMMA GRD)
  (if (string-ci=? (get-lemma (cog-name GRD)) (cog-name LEMMA))
      (stv 1 1)
      (stv 0 1)))

; ----------
(define (phrase STR)
"
  Occurrence of a phrase or a group of words.
  All the words are assumed to be literal / non-canonical.
"
  (fold (lambda (wd lst)
                (list (append (car lst) (car wd))
                      (append (cadr lst) (cadr wd))
                      (append (caddr lst) (caddr wd))))
        (list '() '() '())
        (map word (string-split STR #\sp))))

; ----------
(define (concept STR)
"
  Occurrence of a concept.
"
  (let ((g (Glob (gen-var STR #t)))
        (clength (concept-length (Concept STR))))
    (list (list (TypedVariable g (TypeSet (Type "WordNode")
                                          (Interval (Number 1)
                                                    (Number clength)))))
          (list (Evaluation (GroundedPredicate "scm: ghost-concept?")
                            (List (Concept STR) g)))
          (list g))))

(define-public (ghost-concept? CONCEPT . GRD)
"
  Check if the value grounded for the GlobNode is actually a member
  of the concept.
"
  (cog-logger-debug ghost-logger
    "In ghost-concept? CONCEPT: ~aGRD: ~a" CONCEPT GRD)
  (if (is-member? GRD (get-members CONCEPT))
      (stv 1 1)
      (stv 0 1)))

; ----------
(define (choices TERMS)
"
  Occurrence of a list of choices.
  Existence of either one of the items in the list will be
  considered as a match.
"
  (let ((g (Glob (gen-var "choices" #t)))
        (tlength (term-length TERMS)))
    (list (list (TypedVariable g (TypeSet (Type "WordNode")
                                          (Interval (Number 1)
                                                    (Number tlength)))))
          (list (Evaluation (GroundedPredicate "scm: ghost-choices?")
                            (List (List (terms-to-atomese TERMS)) g)))
          (list g))))

(define-public (ghost-choices? CHOICES . GRD)
"
  Check if the value grounded for the GlobNode is actually a member
  of the list of choices.
"
  (cog-logger-debug ghost-logger
    "In ghost-choices? CHOICES: ~aGRD: ~a" CHOICES GRD)
  (let* ((chs (cog-outgoing-set CHOICES))
         (cpts (append-map get-members (cog-filter 'ConceptNode chs))))
        (if (is-member? GRD (append chs cpts))
            (stv 1 1)
            (stv 0 1))))

; ----------
(define (optionals TERMS)
"
  Occurrence of one or a list of terms that is optional.
"
  (let ((g (Glob (gen-var "optionals" #t)))
        (tlength (term-length TERMS)))
    (list (list (TypedVariable g (TypeSet (Type "WordNode")
                                 (Interval (Number 0)
                                           (Number tlength)))))
          (list (Evaluation (GroundedPredicate "scm: ghost-optionals?")
                            (List (List (terms-to-atomese TERMS)) g)))
          (list g))))

(define-public (ghost-optionals? OPTIONALS . GRD)
"
  Check if the value grounded for the GlobNode is either empty, or
  a member of the list of optional words.
"
  (cog-logger-debug ghost-logger
    "In ghost-optionals? OPTIONALS: ~aGRD: ~a" OPTIONALS GRD)
  (if (nil? GRD)
      (stv 1 1)
      (let* ((opts (cog-outgoing-set OPTIONALS))
             (cpts (append-map get-members (cog-filter 'ConceptNode opts))))
            (if (is-member? GRD (append opts cpts))
                (stv 1 1)
                (stv 0 1)))))

; ----------
(define (negation TERMS)
"
  Absent of one or a list of terms.
"
  (list '()  ; No variable declaration
        (list (Evaluation (GroundedPredicate "scm: ghost-negation?")
                          (List (terms-to-atomese TERMS))))
        ; Nothing for the word-seq
        '()))

(define-public (ghost-negation? . TERMS)
"
  Check if the input sentence has none of the terms specified.
"
  (let* ; Get the raw text input
        ((sent (ghost-get-curr-sent))
         (txt-node (cog-chase-link 'ListLink 'Node sent))
         (rtxt (if (nil? txt-node) "" (cog-name (car txt-node))))
         (ltxt (if (string-null? rtxt) ""
                 (string-join (map get-lemma (string-split rtxt #\sp))))))
        (if (any (lambda (t) (text-contains? rtxt ltxt t)) TERMS)
            (stv 0 1)
            (stv 1 1))))

; ----------
(define (wildcard LOWER UPPER)
"
  Occurrence of a wildcard that the number of atoms to be matched
  can be restricted.
  Note: -1 in the upper bound means infinity.
"
  (let* ((g (Glob (gen-var "wildcard" #t))))
    (list (list
      (TypedVariable g
        (TypeSet (Type "WordNode")
                 (Interval (Number LOWER) (Number UPPER)))))
        '()
        (list g))))

; ----------
(define (tts-feature ARGS)
"
  Occurrence of a TTS feature, like a pause or change of tone etc.
"
  (ExecutionOutput (GroundedSchema "scm: ghost-tts-feature")
                   (List ARGS)))

(define-public (ghost-tts-feature . ARGS)
"
  Support features like |worry,$med,3.0| that exist in the current
  rule base.
  The only reason of parsing and restructing the feature instead
  of just sending it out as a string is because there may be
  variables in it -- e.g. $med, and we will have to get the value
  of it before sending the whole thing out.
  The TTS server will handle the rest afterwards.
"
  ; TODO: Should be handled in OpenCog internally?
  (Word (string-append "|" (string-join (map cog-name (flatten-list ARGS)) ",") "|")))

; ----------
(define (set-delay ARG)
"
  Occurrence of a set-delay event, which will set the STT cutoff time.
"
  (ExecutionOutput (GroundedSchema "scm: ghost-set-stt-cutoff")
                   (List ARG))
)

(define-public (ghost-set-stt-cutoff SECOND)
"
  Support things like {% set delay=2 %} in the action of a rule, which
  set how long STT should wait before sending the input to GHOST.
"
  ; TODO: Should be handled in OpenCog internally?
  (Word (string-append "{% set delay=" (cog-name SECOND) " %}"))
)

; ----------
(define-public (ghost-any-text-input?)
"
  Check if there is any text input to GHOST that is currently being processed.
"
  (if (nil? (ghost-get-curr-sent))
    (stv 0 1)
    (stv 1 1))
)

; ----------
(define (parse-method-name method-name)
"
  Checks if the method-name has a prefix 'scm_' or 'py_'
  and converts it to the corresponding name 'scm:name' or 'py:name'
  to be used as GroundedPredicate name.
  By default scm is used if the method-name does not have the prefix.
"
 (cond
  ((string-prefix? "scm_" method-name)
   (string-append "scm:" (substring method-name 4)))
  ((string-prefix? "py_" method-name)
   (string-append "py:" (substring method-name 3)))
  (else (string-append "scm:" method-name))))

; ----------
(define (context-function NAME ARGS)
"
  Occurrence of a function in the context of a rule.
  The Scheme or Python function named NAME should have already been defined.
  The Python name has 'py_' prefix and Scheme name has 'scm_' or no prefix.
"
  ; TODO: Check to make sure the function has been defined
  (Evaluation (GroundedPredicate (parse-method-name NAME))
              (List (map (lambda (a) (if (equal? 'GlobNode (cog-type a))
                                         (List a) a))
                         ARGS))))

; ----------
(define (action-function NAME ARGS)
"
  Occurrence of a function in the action of a rule.
  The Scheme or Python function named NAME should have already been defined.
  The Python name has 'py_' prefix and Scheme name has 'scm_' or no prefix.
"
  ; TODO: Check to make sure the function has been defined
  (ExecutionOutput (GroundedSchema (parse-method-name NAME))
                   (List (map (lambda (a) (if (equal? 'GlobNode (cog-type a))
                                              (List a) a))
                              ARGS))))

; ----------
(define (action-choices ACTIONS)
"
  Pick one of the ACTIONS randomly.
"
  (ExecutionOutput (GroundedSchema "scm: ghost-pick-action")
                   (Set ACTIONS)))

(define-public (ghost-pick-action ACTIONS)
"
  The actual Scheme function being called by the GroundedSchemaNode
  for picking one of the ACTIONS randomly.
"
  (define os (cog-outgoing-set ACTIONS))
  (list-ref os (random (length os) (random-state-from-platform))))

; ----------
(define (get-var-literals VAR)
"
  Return the value grounded in original words.
"
  (ExecutionOutput (GroundedSchema "scm: ghost-get-original-word")
                   (List VAR)))

(define-public (ghost-get-original-word . WORDS)
"
  Given a list of words, return their original words stored as Values.
"
  (if (any (lambda (w) (or (equal? 'GlobNode (cog-type w))
                           (equal? 'VariableNode (cog-type w))))
           WORDS)
      '()
      (List (map
        (lambda (w)
          (define ori-word (cog-value w ghost-word-original))
          (if (nil? ori-word)
            (begin
              (cog-logger-warn ghost-logger "No original word is found for ~a" w)
              w)
            ori-word))
        WORDS))))

; ----------
(define (get-var-lemmas VAR)
"
  Turn the value grounded for VAR into lemmas.
"
  (ExecutionOutput (GroundedSchema "scm: ghost-get-lemma")
                   (List VAR)))

(define-public (ghost-get-lemma . GRD)
"
  Get the lemma of GRD, where GRD can be one or more WordNodes.
"
  (if (any (lambda (g) (or (equal? 'GlobNode (cog-type g))
                           (equal? 'VariableNode (cog-type g))))
           GRD)
      '()
      (List (map
        (lambda (w) (Word (get-lemma (cog-name w))))
        (map (lambda (g) (cog-value g ghost-word-original)) GRD)))))

; ----------
(define (get-user-variable UVAR)
"
  Get the value of a user variable.
"
  (ExecutionOutput (GroundedSchema "scm: ghost-get-user-variable")
                   (List (ghost-uvar UVAR))))

(define-public (ghost-get-user-variable UVAR)
"
  Get the value stored for VAR.
"
  (define grd (assoc-ref uvars UVAR))
  (if (equal? grd #f) (List) grd))

; ----------
(define (set-user-variable UVAR VAL)
"
  Assign a string value to a user variable.
"
  (ExecutionOutput (GroundedSchema "scm: ghost-set-user-variable")
                   (List (ghost-uvar UVAR) (List VAL))))

(define-public (ghost-set-user-variable UVAR VAL)
"
  Assign VAL to UVAR.
"
  ; In some (rare) cases VAL may be null as the glob is grounded
  ; to nothing legitimately, so check the rule pattern as there
  ; are likely more than one ways of grounding it
  (if (equal? 0 (cog-arity VAL))
    (cog-logger-warn ghost-logger
      "Trying to set user variable ~a but the value is NULL!" UVAR)
    (set! uvars (assoc-set! uvars UVAR VAL)))
  (True))

; ----------
(define (uvar-eval UVAR)
"
  Check if a user variable has been defined and the value is checked against
  the input.
"
  (let* ((g (Glob (gen-var (string-append "user-variable-" UVAR "-grounding") #f)))
         (v (list (TypedVariable g
                    (TypeSet
                      (Type "WordNode")
                      (Interval (Number 1) (Number -1))))))
         (c (list (compare-equal (get-user-variable UVAR) g))))
    (list v c (list g) (list g))))

; ----------
(define-public (ghost-execute-base-action . ACTIONS)
"
  Execute the actions and record the results.
"
  (define txt-str "")
  (define txt-atoms '())
  (define atoms-created '())
  (define (extract actions)
    (for-each
      (lambda (a)
        (cond ((or (equal? 'ListLink (cog-type a))
                   (equal? 'SetLink (cog-type a)))
               (extract (cog-outgoing-set a)))
              ((or (equal? 'WordNode (cog-type a))
                   ; This assumes that the name of a ConceptNode
                   ; is something meaningful to say...
                   (equal? 'ConceptNode (cog-type a)))
               (set! txt-atoms (append txt-atoms (list a)))
               (set! txt-str (string-trim (string-append txt-str " " (cog-name a)))))
              ; These can be ignored, may just be the return
              ; of a GroundedSchemaNode
              ((or (equal? 'TrueLink (cog-type a))
                   (equal? 'FalseLink (cog-type a)))
               '())
              (else (set! atoms-created (append atoms-created (list a))))))
      actions))
  ; See what needs to be handled
  (extract ACTIONS)
  ; Is there anything to say?
  (if (not (string-null? txt-str))
      (begin (cog-execute!
        (Put (DefinedSchema "say") (List (Node txt-str) (Concept ""))))))
  ; New atoms being created
  (if (not (nil? atoms-created))
      (cog-logger-debug ghost-logger "Atoms Created: ~a" atoms-created))
  ; Record the result
  (set! ghost-result (append txt-atoms atoms-created))
  ; Return an atom
  (True))

; ----------
(define-public (ghost-execute-action . ACTIONS)
"
  Execute the actions and update the internal state -- that particular
  input will no longer triggered any other GHOST rules.
"
  (apply ghost-execute-base-action ACTIONS)

  ; Reset the state
  (State ghost-curr-proc (Concept "Default State")))

; ----------
(define (compare-equal LV RV)
  (Evaluation
    (GroundedPredicate "scm: ghost-compare-equal?")
    (List (List LV) (List RV)))
)

(define (compare-not-equal LV RV)
  (Evaluation
    (GroundedPredicate "scm: ghost-compare-not-equal?")
    (List (List LV) (List RV)))
)

(define (compare-smaller LV RV)
  (Evaluation
    (GroundedPredicate "scm: ghost-compare-smaller?")
    (List (List LV) (List RV)))
)

(define (compare-smaller-equal LV RV)
  (Evaluation
    (GroundedPredicate "scm: ghost-compare-smaller-equal?")
    (List (List LV) (List RV)))
)

(define (compare-greater LV RV)
  (Evaluation
    (GroundedPredicate "scm: ghost-compare-greater?")
    (List (List LV) (List RV)))
)

(define (compare-greater-equal LV RV)
  (Evaluation
    (GroundedPredicate "scm: ghost-compare-greater-equal?")
    (List (List LV) (List RV)))
)

(define-public (ghost-compare-equal? LV RV)
  (cond
    ((and (not (nil? (gar RV)))
          (equal? 'ConceptNode (cog-type (gar RV)))
          (is-member?
            (flatten-list (cog-outgoing-set LV))
            (get-members (gar RV))))
     (stv 1 1))
    ((compare "equal" LV RV) (stv 1 1))
    (else (stv 0 1)))
)

(define-public (ghost-compare-not-equal? LV RV)
  (define result (ghost-compare-equal? LV RV))
  (if (equal? result (stv 1 1))
    (stv 0 1)
    (stv 1 1))
)

(define-public (ghost-compare-smaller? LV RV)
  (if (compare "smaller" LV RV)
    (stv 1 1)
    (stv 0 1))
)

(define-public (ghost-compare-smaller-equal? LV RV)
  (if (compare "smaller_equal" LV RV)
    (stv 1 1)
    (stv 0 1))
)

(define-public (ghost-compare-greater? LV RV)
  (if (compare "greater" LV RV)
    (stv 1 1)
    (stv 0 1))
)

(define-public (ghost-compare-greater-equal? LV RV)
  (if (compare "greater_equal" LV RV)
    (stv 1 1)
    (stv 0 1))
)

(define (compare OPERATOR LV RV)
  (define lv
    (if (equal? 'ListLink (cog-type LV))
      (flatten-list (cog-outgoing-set LV))
      (list LV)))
  (define rv
    (if (equal? 'ListLink (cog-type RV))
      (flatten-list (cog-outgoing-set RV))
      (list RV)))
  (define lv-str (string-join (map cog-name lv)))
  (define rv-str (string-join (map cog-name rv)))
  (define lv-num
    (string->number (string-join (map cog-name lv) "")))
  (define rv-num
    (string->number (string-join (map cog-name rv) "")))
  (define both-numbers? (and lv-num rv-num))

  (cond
    ((string=? "equal" OPERATOR)
     (if both-numbers?
       (= lv-num rv-num)
       (string-ci=? lv-str rv-str)))
    ((string=? "smaller" OPERATOR)
     (and both-numbers?
          (< lv-num rv-num)))
    ((string=? "smaller_equal" OPERATOR)
     (and both-numbers?
          (<= lv-num rv-num)))
    ((string=? "greater" OPERATOR)
     (and both-numbers?
          (> lv-num rv-num)))
    ((string=? "greater_equal" OPERATOR)
     (and both-numbers?
          (>= lv-num rv-num)))
    (else #f))
)
