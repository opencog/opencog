;; Assorted functions for translating individual terms into Atomese fragments.
;; A term is like a "feature" that can be used when creating a GHOST rule.

; ----------
(define (word STR)
"
  Occurrence of a word, a word that should be matched literally.
"
  (let* ((v1 (WordNode STR))
         (v2 (Variable (gen-var STR #f)))
         (l (WordNode (get-lemma STR)))
         (v (list (TypedVariable v2 (Type "WordInstanceNode"))))
         (c (list (WordInstanceLink v2 (Variable "$P"))
                  (ReferenceLink v2 v1))))
    (list v c (list v1) (list l))))

; ----------
(define (word-apos STR)
"
  Occurrence of a word with an apostrophe, e.g. I'm, it's etc.
  Should be matched literally.
"
  (let* (; This turns ’ into ' just to treat them as the same thing
         (nstr (regexp-substitute/global #f "’" STR 'pre "'" 'post))
         (v1 (Variable (gen-var STR #f)))
         (l (WordNode nstr))
         (v (list (TypedVariable v1 (Type "WordNode")))))
    (list v (list) (list v1) (list l))))

; ----------
(define (lemma STR)
"
  Lemma occurrence, aka canonical form of a term.
  This is the default for word mentions in the rule pattern.
"
  (let* ((v1 (Variable (gen-var STR #t)))
         (v2 (Variable (gen-var STR #f)))
         (l (WordNode (get-lemma STR)))
         (v (list (TypedVariable v1 (Type "WordNode"))
                  (TypedVariable v2 (Type "WordInstanceNode"))))
         (c (list (ReferenceLink v2 v1)
                  ; In some rare situation, particularly if the input
                  ; sentence is not grammatical, RelEx may not lemmatize a
                  ; word because of the ambiguity
                  ; So just to be sure "l" is the stem of "v1",
                  ; a GroundedPredicateNode is used instead of putting
                  ; "(LemmaLink v2 l)" in the context
                  (Evaluation (GroundedPredicate "scm: ghost-lemma?")
                              (List v1 l))
                  (WordInstanceLink v2 (Variable "$P")))))
    (list v c (list v1) (list l))))

(define-public (ghost-lemma? GRD LEMMA)
"
  Check if LEMMA is the stem of GRD.
"
  (cog-logger-debug ghost-logger
    "In ghost-lemma? LEMMA: ~aGRD: ~a" LEMMA GRD)
  (if (equal? (get-lemma (cog-name GRD)) (cog-name LEMMA))
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
                      (append (caddr lst) (caddr wd))
                      (append (cadddr lst) (cadddr wd))))
        (list '() '() '() '())
        (map word (string-split STR #\sp))))

; ----------
(define (concept STR)
"
  Occurrence of a concept.
"
  (let ((g1 (Glob (gen-var STR #t)))
        (g2 (Glob (gen-var STR #f)))
        (clength (concept-length (Concept STR))))
    (list (list (TypedVariable g1 (TypeSet (Type "WordNode")
                                           (Interval (Number 1)
                                                     (Number clength))))
                (if ghost-with-ecan
                  (list)
                  (TypedVariable g2 (TypeSet (Type "WordNode")
                                             (Interval (Number 1)
                                                       (Number clength))))))
          (list (Evaluation (GroundedPredicate "scm: ghost-concept?")
                            (List (Concept STR) g1)))
          (list g1)
          (list g2))))

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
  (let ((g1 (Glob (gen-var "choices" #t)))
        (g2 (Glob (gen-var "choices" #f)))
        (tlength (term-length TERMS)))
    (list (list (TypedVariable g1 (TypeSet (Type "WordNode")
                                           (Interval (Number 1)
                                                     (Number tlength))))
                (if ghost-with-ecan
                  (list)
                  (TypedVariable g2 (TypeSet (Type "WordNode")
                                             (Interval (Number 1)
                                                       (Number tlength))))))
          (list (Evaluation (GroundedPredicate "scm: ghost-choices?")
                            (List (List (terms-to-atomese TERMS)) g1)))
          (list g1)
          (list g2))))

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
  (let ((g1 (Glob (gen-var "optionals" #t)))
        (g2 (Glob (gen-var "optionals" #f)))
        (tlength (term-length TERMS)))
    (list (list (TypedVariable g1 (TypeSet (Type "WordNode")
                                  (Interval (Number 0)
                                            (Number tlength))))
                (if ghost-with-ecan
                  (list)
                  (TypedVariable g2 (TypeSet (Type "WordNode")
                                    (Interval (Number 0)
                                              (Number tlength))))))
          (list (Evaluation (GroundedPredicate "scm: ghost-optionals?")
                            (List (List (terms-to-atomese TERMS)) g1)))
          (list g1)
          (list g2))))

(define-public (ghost-optionals? OPTIONALS . GRD)
"
  Check if the value grounded for the GlobNode is either empty, or
  a member of the list of optional words.
"
  (cog-logger-debug ghost-logger
    "In ghost-optionals? OPTIONALS: ~aGRD: ~a" OPTIONALS GRD)
  (if (null? GRD)
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
        ; Nothing for the word-seq and lemma-seq
        '() '()))

(define-public (ghost-negation? . TERMS)
"
  Check if the input sentence has none of the terms specified.
"
  (let* ; Get the raw text input
        ((sent (ghost-get-curr-sent))
         (rtxt (cog-name (car (cog-chase-link 'ListLink 'Node sent))))
         (ltxt (string-join (map get-lemma (string-split rtxt #\sp)))))
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
  (let* ((g1 (Glob (gen-var "wildcard" #t)))
         (g2 (Glob (gen-var "wildcard" #f))))
    (list (list
      (TypedVariable g1
        (TypeSet (Type "WordNode")
                 (Interval (Number LOWER) (Number UPPER))))
      (if ghost-with-ecan
        (list)
        (TypedVariable g2
          (TypeSet (Type "WordNode")
                   (Interval (Number LOWER) (Number UPPER))))))
        '()
        (list g1)
        (list g2))))

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
(define (context-function NAME ARGS)
"
  Occurrence of a function in the context of a rule.
  The Scheme function named NAME should have already been defined.
"
  ; TODO: Check to make sure the function has been defined
  (Evaluation (GroundedPredicate (string-append "scm: " NAME))
              (List (map (lambda (a) (if (equal? 'GlobNode (cog-type a))
                                         (List a) a))
                         ARGS))))

; ----------
(define (action-function NAME ARGS)
"
  Occurrence of a function in the action of a rule.
  The Scheme function named NAME should have already been defined.
"
  ; TODO: Check to make sure the function has been defined
  (ExecutionOutput (GroundedSchema (string-append "scm: " NAME))
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
      (List (map Word (map get-lemma (map cog-name GRD))))))

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
(define (uvar-exist? UVAR)
"
  Check if a user variable has been defined.
"
  (Evaluation (GroundedPredicate "scm: ghost-user-variable-exist?")
              (List (ghost-uvar UVAR))))

(define-public (ghost-user-variable-exist? UVAR)
"
  Check if UVAR has been defined.
"
  (if (equal? (assoc-ref uvars UVAR) #f)
      (stv 0 1)
      (stv 1 1)))

; ----------
(define (uvar-equal? UVAR VAL)
"
  Check if the value of the user variable VAR equals to VAL.
"
  ; TODO: VAL can also be a concept etc?
  (Evaluation (GroundedPredicate "scm: ghost-user-variable-equal?")
              (List (ghost-uvar UVAR) (List (Word VAL)))))

(define-public (ghost-user-variable-equal? UVAR VAL)
"
  Check if the value of UVAR equals VAL.
"
  (if (equal? (assoc-ref uvars UVAR) VAL)
      (stv 1 1)
      (stv 0 1)))

; ----------
(define-public (ghost-execute-action . ACTIONS)
"
  Execute the actions and update the internal state.
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
      (begin (cog-logger-info ghost-logger "Say: \"~a\"" txt-str)
             (cog-execute! (Put (DefinedPredicate "Say") (Node txt-str)))))
  ; New atoms being created
  (if (not (null? atoms-created))
      (cog-logger-info ghost-logger "Atoms Created: ~a" atoms-created))
  ; Record the result
  (set! ghost-result (append txt-atoms atoms-created))
  ; Reset the state
  (State ghost-curr-proc (Concept "Default State")))

; ----------
(define-public (ghost-update-rule-strength RULENAME VALUE)
"
  Update the TruthValue strength of the rule with alias RULENAME to VALUE.
"
  ; Get the action of the rule with alias RULENAME, and then
  ; find all the rules that also contains this same action
  ; TODO: Better if the label the action instead of the rule?
  (define rules (get-related-psi-rules (psi-get-action
    (car (cog-chase-link 'ListLink 'ImplicationLink RULENAME)))))

  ; Get the value
  (define val (string->number (cog-name VALUE)))

  ; Update the TVs
  (for-each
    (lambda (rule)
      (cog-set-tv! rule (cog-new-stv val (cog-stv-confidence rule))))
    rules)

  ; Return an atom
  (True))
