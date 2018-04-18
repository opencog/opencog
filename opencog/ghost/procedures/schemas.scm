; Action APIs and schemas

; --------------------------------------------------------------
; APIs for forming GroundedSchemas that are use for exectuing
; actions.
; NOTE: For testing use (opencog eva-behavior) module. For
; running use (opencog movement) module. This is because the
; APIs are atomese DefinedPredicates.
; TODO: List out the DefinedPredicates that are used as API, so as to
; use delete-definition. Also adapt the scheme function naming convention
; to make remembering easier.
; --------------------------------------------------------------
(define fini (Node "finished-action"))

(define (animation emotion gesture)
  ;TODO: Remove this hack.
  (let* ((e (cog-name emotion))
    (g (cog-name gesture))
    (temp-gesture (if (equal? "nod" g) "nod-1" g)))
  (cog-evaluate!
    (Put
      (DefinedPredicate "Show class gesture")
      (List
        (Concept e)
        (Concept temp-gesture))))

     fini
   )
)

(define (expression expression-type)
  (let ((e (cog-name expression-type)))
    (cog-evaluate!
      (Put
        (DefinedPredicate "Show class expression")
        (List
          (Concept "neutral-keep-alive")
          (Concept e))))
    fini
  )
)

(define* (start_timer #:optional (timer-id (Concept "Default-Timer")))
"
  start_timer TIMER-ID (optional)

  Record the current time for TIMER-ID.
  If TIMER-ID is not given, a default timer will be used.
"
  (set-time-perceived! timer-id)
  fini
)

(define (decrease_urge goal value)
"
  decrease_urge GOAL VALUE

  Decrease the urge of GOAL by VALUE.
"
  (psi-decrease-urge (Concept (cog-name goal))
    (string->number (cog-name value)))
)

(define (increase_urge goal value)
"
  increase_urge GOAL VALUE

  Increase the urge of GOAL by VALUE.
"
  (psi-increase-urge (Concept (cog-name goal))
    (string->number (cog-name value)))
)

(define (stimulate_words . words)
"
  stimulate_words WORDS

  Stimulate the WordNodes corresponding to WORDS.
"
  (for-each
    (lambda (w) (cog-stimulate (Word (cog-name w)) 200))
    words)
  fini
)

(define (stimulate_concepts . concepts)
"
  stimulate_concepts CONCEPTS

  Stimulate the ConceptNodes corresponding to CONCEPTS.
"
  (for-each
    (lambda (c) (cog-stimulate (Concept (cog-name c)) 200))
    concepts)
  fini
)

(define (stimulate_rules . rule-labels)
"
  stimulate_rules RULE-LABELS

  Stimulate the rules with RULE-LABELS.
"
  (for-each
    (lambda (r) (cog-stimulate (get-rule-from-alias (cog-name r)) 200))
    rule-labels)
  fini
)

(define (set_rule_sti rule-label val)
"
  set_rule_sti RULE-LABEL VAL

  Set the STI of a rule with RULE-LABEL to VAL.
"
  (cog-set-sti!
    (get-rule-from-alias (cog-name rule-label))
    (string->number (cog-name val)))
)
