; Perception APIs and predicates

; --------------------------------------------------------------
; APIs for forming GroundedPredicates that are used for
; checking if the world is in a particular state or not.
; --------------------------------------------------------------
(define (any-face-seen?)
  (define get-models
    (Get
      (TypedVariable
        (Variable "seen-faces")
        (Signature
          (Evaluation
            (Predicate "see")
            (List
              (Concept "I")
              (Type "ConceptNode")))))
      (And
        (Evaluation
          (GroundedPredicate "scm: is-model-true?")
          (List
            (Variable "seen-faces")))
        (Variable "seen-faces"))))

  (let ((models (cog-outgoing-set (cog-execute! get-models))))
    (if (null? models)
      (stv 0 1)
      (stv 1 1)
    )
  )
)

(define (any-person-emotion? emotion-type)
  (define get-models
    (Get
      (TypedVariable
        (Variable "face-emotion")
        (Signature
          (Evaluation
            (Predicate emotion-type)
            (List
              (Type "ConceptNode")))))
      (And
        (Evaluation
          (GroundedPredicate "scm: is-model-true?")
          (List
            (Variable "face-emotion")))
        (Variable "face-emotion"))))

  (let ((models (cog-outgoing-set (cog-execute! get-models))))
    (if (null? models)
      (stv 0 1)
      (stv 1 1)
    )
  )
)

(define (any-person-talking?)
  (define get-models
    (Get
      (TypedVariable
        (Variable "face-talking")
        (Signature
          (Evaluation
            (Predicate "talking")
            (List
              (Type "ConceptNode")))))
      (And
        (Evaluation
          (GroundedPredicate "scm: is-model-true?")
          (List
            (Variable "face-talking")))
        (Variable "face-talking"))))

  (let ((models (cog-outgoing-set (cog-execute! get-models))))
    (if (null? models)
      (stv 0 1)
      (stv 1 1)
    )
  )
)

; TODO: If the stream of sensory inputs are interupted, for whatever reason,
; then the variations in the confidence value are not updated and thus the
; state of the world wouldn't be correct. To fix this add a time window
; similar to word_perceived. If the time-window is passed then it returns
; false.

;(define (person_appears face-id)
;  (cog-pointmem-get-locs-of-atom facemap (Concept face-id))
;)

(define* (person_appears #:optional face-id)
  (if face-id
    (is-model-true? (see-face (cog-name face-id)))
    (any-face-seen?)
  )
)

(define* (person_smiles #:optional face-id)
  (if face-id
    (is-model-true? (face-emotion (cog-name face-id) "smile"))
    (any-person-emotion? "smile")
  )
)

(define* (person_angry #:optional face-id)
  (if face-id
    (is-model-true? (face-emotion (cog-name face-id) "angry"))
    (any-person-emotion? "angry")
  )
)

(define* (person_talking #:optional face-id)
  (if face-id
    (is-model-true? (face-talking (cog-name face-id)))
    (any-person-talking?)
  )
)

(define* (person_not_talking #:optional face-id)
  (negate-stv! (person_talking face-id))
)

(define* (word_perceived word #:optional time-interval)
  (if time-interval
    (was-perceived? word (current-time-us)
      (string->number (cog-name time-interval)))
    (was-perceived? word (current-time-us) 0.01)
  )
)

(define* (after_min minutes #:optional (timer-id (Concept "Default-Timer")))
"
  after_min MINUTES TIMER-ID (optional)

  Returns (stv 1 1) if current time >= the timer's start time (if given) + MINUTES.
  Otherwise, returns (stv 0 1)
"
  (define t (time-perceived timer-id))

  ; If it's null, the timer probably has not started yet
  (if (null? t)
    (stv 0 1)
    (if (>= (current-time-us)
            (+ t (* (string->number (cog-name minutes)) 60)))
        (stv 1 1)
        (stv 0 1)))
)

(define (after_user_started_talking secs)
"
  after_user_started_talking SECS

  Returns (stv 1 1) if current time >= the time any user started talking plus
  SECS. Otherwise, returns (stv 0 1).
"
  (if (since-true-transition-occurred? face-talking-sign
    (string->number (cog-name secs)))
    (stv 1 1)
    (stv 0 1)
  )
)

(define (after_user_stopped_talking secs)
"
  after_user_stopped_talking SECS

  Returns (stv 1 1) if current time >= the time any user-stopped talking plus
  SECS. Otherwise, returns (stv 0 1).
"
  (if (since-false-transition-occurred? face-talking-sign
    (string->number (cog-name secs)))
    (stv 1 1)
    (stv 0 1)
  )
)

; Create the GroundedPredicateNode, and link it to a generic "timer-predicate"
; so that we can stimulate the generic one and the STI will diffuse to
; the specific predicates connecting to it
(Inheritance (GroundedPredicate "scm: after_min") (Concept "timer-predicate"))
(Inheritance (GroundedPredicate "scm: after_user_started_talking")
  (Concept "timer-predicate"))
(Inheritance (GroundedPredicate "scm: after_user_stopped_talking")
  (Concept "timer-predicate"))
