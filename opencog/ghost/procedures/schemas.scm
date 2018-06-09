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

(DefineLink
  (DefinedSchema "say")
  (LambdaLink
    (VariableList
      (Variable "sentence")
      (Variable "fallback-id"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "say")
        (Variable "sentence")))
  )
)

(define (fallback_on fallback-id)
"
  fallback_on  FALLBACK-ID

  Use the fallback system identified by FALLBACK-ID
"
  (cog-execute! (Put (DefinedSchema "say") (List (Concept "") fallback-id)))
  fini
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "say-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
       (List (Concept "shutup")))
  )
)

(define (shutup)
  (cog-execute! (Put (DefinedSchema "say-cancel") (List)))
  fini
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "gaze-at")
  (LambdaLink
    (VariableList
      (Variable "face-id")
      (Variable "speed"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "gaze-at")
        (Variable "face-id")
        (Variable "speed")))
  )
)

(define (gaze_at face-id speed)
  (cog-execute! (Put (DefinedSchema "gaze-at") (List face-id speed)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "gaze-at-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "gaze-at-cancel")))
  )
)

(define (gaze_at_cancel)
  (cog-execute! (Put (DefinedSchema "gaze-at-cancel") (List)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "blink")
  (LambdaLink
    (VariableList
      (Variable "mean")
      (Variable "variation"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "blink")
        (Variable "mean")
        (Variable "variation")))
  )
)

(define (blink mean variation)
  (cog-execute! (Put (DefinedSchema "blink") (List mean variation)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "blink-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "blink-cancel")))
  )
)

(define (blink_cancel)
  (cog-execute! (Put (DefinedSchema "blink-cancel") (List)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "emote")
  (LambdaLink
    (VariableList
      (Variable "name")
      (Variable "magnitude")
      (Variable "duration")
      (Variable "blend"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "emote")
        (Variable "name")
        (Variable "magnitude")
        (Variable "duration")
        (Variable "blend")))
  )
)

(define (emote name magni duration blend)
  (cog-execute!
    (Put (DefinedSchema "emote") (List name magni duration blend)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "gesture")
  (LambdaLink
    (VariableList
      (Variable "name")
      (Variable "speed")
      (Variable "magnitude")
      (Variable "repeat"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "gesture")
        (Variable "name")
        (Variable "speed")
        (Variable "magnitude")
        (Variable "repeat")))
  )
)

(define (gesture name speed magni repeat)
  (cog-execute!
    (Put (DefinedSchema "gesture") (List name speed magni repeat)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "soma")
  (LambdaLink
    (VariableList
      (Variable "name")
      (Variable "magnitude")
      (Variable "rate")
      (Variable "ease_in"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "soma")
        (Variable "name")
        (Variable "magnitude")
        (Variable "rate")
        (Variable "ease_in")))
  )
)

(define (soma name magni rate ease-in)
  (cog-execute!
    (Put (DefinedSchema "soma") (List name magni rate ease-in)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "soma-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "soma-cancel")))
  )
)

(define (soma_cancel)
  (cog-execute! (Put (DefinedSchema "soma-cancel") (List)))
)

; --------------------------------------------------------------
(DefineLink
  (DefinedSchema "saccade")
  (LambdaLink
    (VariableList
      (Variable "mean")
      (Variable "variation")
      (Variable "paint_scale")
      (Variable "eye_size")
      (Variable "eye_distance")
      (Variable "mouth_width")
      (Variable "mouth_height")
      (Variable "weight_eyes")
      (Variable "weight_mouth"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "saccade")
        (Variable "mean")
        (Variable "variation")
        (Variable "paint_scale")
        (Variable "eye_size")
        (Variable "eye_distance")
        (Variable "mouth_width")
        (Variable "mouth_height")
        (Variable "weight_eyes")
        (Variable "weight_mouth")))
  )
)

(define (saccade-args . nums)
  (List (map (lambda (x) (Number x)) nums ))
)

(define (saccade_explore)
  (cog-execute! (Put (DefinedSchema "saccade")
    (saccade-args 0.8 0.3 1.0 15.0 100.0 90.0 27.0 0.8 0.2))
  )
)

(define (saccade_listen)
  (cog-execute! (Put (DefinedSchema "saccade")
    (saccade-args 1.0 0.6 0.3 11.0 80.0 50.0 13.0 0.8 0.2))
  )
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "saccade-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "saccade-cancel")))
  )
)

(define (saccade_cancel)
  (cog-execute! (Put (DefinedSchema "saccade-cancel") (List)))
)

; --------------------------------------------------------------
(define* (start_timer #:optional (timer-id (Concept "Default-Timer")))
"
  start_timer TIMER-ID (optional)

  Record the current time for TIMER-ID.
  If TIMER-ID is not given, a default timer will be used.
"
  (set-time-perceived! (Concept (cog-name timer-id))
    (FloatValue (current-time-us)))
  fini
)

(define (decrease_urge goal value)
"
  decrease_urge GOAL VALUE

  Decrease the urge of GOAL by VALUE.
"
  (psi-decrease-urge (Concept (cog-name goal))
    (string->number (cog-name value)))
  fini
)

(define (increase_urge goal value)
"
  increase_urge GOAL VALUE

  Increase the urge of GOAL by VALUE.
"
  (define goal-node (Concept (cog-name goal)))
  (define related-psi-rules
    (filter psi-rule? (cog-incoming-set goal-node)))

  (psi-increase-urge goal-node
    (string->number (cog-name value)))

  ; Stimulate the rules associate with this goal
  (for-each
    (lambda (r)
      (cog-stimulate r default-stimulus))
    related-psi-rules)

  fini
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

(define (max_sti_words . words)
"
  max_sti_words WORDS

  Maximize the STI of the WordNodes correspondings to WORDS.
"
  (define max-sti (cog-av-sti (car (cog-af 1))))
  (for-each
    (lambda (w) (cog-set-sti! (Word (cog-name w)) max-sti))
    words)
  fini
)

(define (max_sti_concepts . concepts)
"
  max_sti_concepts . CONCEPTS

  Maximize the STI of the ConceptNodes corresponding to CONCEPTS.
"
  (define max-sti (cog-av-sti (car (cog-af 1))))
  (for-each
    (lambda (c) (cog-set-sti! (Concept (cog-name c)) max-sti))
    concepts)
  fini
)

(define (max_sti_rules . rule-labels)
"
  max_sti_rules . RULE-LABELS

  Maximize the STI of the rules with RULE-LABELS.
"
  (define max-sti (cog-av-sti (car (cog-af 1))))
  (for-each
    (lambda (r) (cog-set-sti! (get-rule-from-alias (cog-name r)) max-sti))
    rule-labels)
  fini
)

(define (min_sti_words . words)
"
  min_sti_words WORDS

  Minimize the STI of the WordNodes correspondings to WORDS.
"
  (for-each
    (lambda (w) (cog-set-sti! (Word (cog-name w)) 0))
    words)
  fini
)

(define (min_sti_concepts . concepts)
"
  min_sti_concepts . CONCEPTS

  Minimize the STI of the ConceptNodes corresponding to CONCEPTS.
"
  (for-each
    (lambda (c) (cog-set-sti! (Concept (cog-name c)) 0))
    concepts)
  fini
)

(define (min_sti_rules . rule-labels)
"
  min_sti_rules . RULE-LABELS

  Minimize the STI of the rules with RULE-LABELS.
"
  (for-each
    (lambda (r) (cog-set-sti! (get-rule-from-alias (cog-name r)) 0))
    rule-labels)
  fini
)
