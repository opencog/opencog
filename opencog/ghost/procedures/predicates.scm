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
        (Evaluation
          (GroundedPredicate "scm: was-perceived?")
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
        (Evaluation
          (GroundedPredicate "scm: was-perceived?")
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

; --------------------------------------------------------------
(define (true-perception-occuring? model)
"
  true-perception-occuring? MODEL

  Returns (stv 1 1) if the perception-model MODEL is true within the
  default-time-interval, otherwise it returns (stv 0 1).
"
  (if (equal? (stv 1 1) (was-perceived? model))
    (is-model-true? model)
    (stv 0 1)
  )
)

(define (true-event-occuring? model)
"
  true-event-occuring? MODEL

  Returns (stv 1 1) if the MODEL made a true transition within the last
  event-period time, otherwise returns (stv 0 1).
"
  (if (true-transition-occurring? model event-period)
    (stv 1 1)
    (stv 0 1)
  )
)

(define (false-event-occuring? model)
"
  false-event-occuring? MODEL

  Returns (stv 1 1) if the MODEL made a false transition within the last
  event-period time, otherwise returns (stv 0 1).
"
  (if (false-transition-occurring? model event-period)
    (stv 1 1)
    (stv 0 1)
  )
)

(define (since-event-started-occuring? model secs)
"
  since-event-started-occuring? MODEL SECS

  An atomese wrapper of the function since-true-transition-occurred?
"
  (if (since-true-transition-occurred? model (string->number (cog-name secs)))
    (stv 1 1)
    (stv 0 1)
  )
)

(define (since-event-stopped-occuring? model secs)
"
  since-event-stopped-occuring? MODEL SECS

  An atomese wrapper of the function since-false-transition-occurred?
"
  (if (since-false-transition-occurred? model (string->number (cog-name secs)))
    (stv 1 1)
    (stv 0 1)
  )
)

; --------------------------------------------------------------
(define-syntax-rule
  (define-face-predicates model-func predicate-node
    t-transitioning? t-occuring? since-t?
    f-transitioning? f-occuring? since-f?)
  ; The definitons are not public so as to be able to control which
  ; of them are exported by this module.
  (begin
    (define* (t-transitioning? #:optional (face-id any-node))
      (true-event-occuring?  (model-func (cog-name face-id))))
    (Inheritance
      (GroundedPredicate (format #f "scm: ~a" 't-transitioning?))
       predicate-node)

    (define* (t-occuring? #:optional (face-id any-node))
      (true-perception-occuring? (model-func (cog-name face-id))))
    (Inheritance
      (GroundedPredicate (format #f "scm: ~a" 't-occuring?))
       predicate-node)

    ; FIXME: This has issues when the window of perception (dti) is passed.
    ; Just because we don't know it doesn't mean it is true
    (define* (since-t? secs #:optional (face-id any-node))
      (since-event-started-occuring? (model-func (cog-name face-id)) secs))
    (Inheritance
      (GroundedPredicate (format #f "scm: ~a" 'since-t?))
       predicate-node)

    (define* (f-transitioning? #:optional (face-id any-node))
      (false-event-occuring? (model-func (cog-name face-id))))
    (Inheritance
      (GroundedPredicate (format #f "scm: ~a" 'f-transitioning?))
       predicate-node)

    (define* (f-occuring? #:optional (face-id any-node))
      (negate-stv! (t-occuring? face-id)))
    (Inheritance
      (GroundedPredicate (format #f "scm: ~a" 'f-occuring?))
       predicate-node)

    ; FIXME: This has issues when the window of perception (dti) is passed.
    ; Just because we don't know it doesn't mean it is true
    (define* (since-f? secs  #:optional (face-id any-node))
      (since-event-stopped-occuring? (model-func (cog-name face-id)) secs))
    (Inheritance
      (GroundedPredicate (format #f "scm: ~a" 'since-f?))
       predicate-node)
  )
)

; --------------------------------------------------------------
; Define predicates for face-talking
(define-face-predicates face-talking face-talking-predicate
   new_talking
   talking
   after_user_started_talking
   end_talking
   not_talking
   after_user_stopped_talking
)

(set-procedure-property! new_talking 'documentation
"
  new_talking [FACE-ID]

  Return (stv 1 1) if the face identified by FACE-ID is starting to talk
  within the last event-period, otherwise returns (stv 0 1).

  IF FACE-ID is not passed then the return value is for any person.
"
)

(set-procedure-property! talking 'documentation
"
  talking [FACE-ID]

  Check if face with FACE-ID is talking.

  IF FACE-ID is not passed then the return value is for any person.
"
)

(set-procedure-property! after_user_started_talking 'documentation
"
  after_user_started_talking SECS [FACE-ID]

  Returns (stv 1 1) if current time >= the time that the user identified by
  FACE-ID started talking + SECS. Otherwise, returns (stv 0 1).

  IF FACE-ID is not passed then the return value is for any person.
"
)

(set-procedure-property! end_talking 'documentation
"
  end_talking [FACE-ID]

  Return (stv 1 1) if the face identified by FACE-ID is stopping to talk
  within the last event-period, otherwise returns (stv 0 1).

  IF FACE-ID is not passed then the return value is for any person.
"
)

(set-procedure-property! not_talking 'documentation
"
  not_talking [FACE-ID]

  Check if face with FACE-ID is not talking.

  IF FACE-ID is not passed then the return value is for any person.
"
)

(set-procedure-property! after_user_stopped_talking 'documentation
"
  after_user_stopped_talking SECS [FACE-ID]

  Returns (stv 1 1) if current time >= the time that the user identified by
  FACE-ID stopped talking + SECS. Otherwise, returns (stv 0 1).

  IF FACE-ID is not passed then the return value is for any person.
"
)

; --------------------------------------------------------------
; Define predicates for face-visiblity
(define-face-predicates see-face see-face-predicate
   new_face
   face
   visible_for
   end_face
   not_visible
   not_visible_for
)

(set-procedure-property! face 'documentation
"
  face [FACE-ID]

  Check if face with FACE-ID was seen.

  IF FACE-ID is not passed then the return value is for any person.
"
)

; --------------------------------------------------------------
(define* (emotion emotion-type #:optional (face-id any-node))
"
  emotion EMOTION-TYPE [FACE-ID]

  Check if face with FACE-ID was seen to have EMOTION-TYPE emotion.
  Returns (stv 1 1) if the the model associated with FACE-ID is true,
  within the default-time-interval, otherwise it returns (stv 0 1).

  IF FACE-ID is not passed then the return value is for any person.
"
  (true-perception-occuring?
    (face-emotion (cog-name face-id) (cog-name emotion-type)))
)

; --------------------------------------------------------------
(define* (word_perceived word #:optional (time-interval dti-node))
  (was-perceived? (Word (cog-name word)) time-interval)
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


; Create the GroundedPredicateNode, and link it to a generic "timer-predicate"
; so that we can stimulate the generic one and the STI will diffuse to
; the specific predicates connecting to it
(Inheritance (GroundedPredicate "scm: after_min") (Concept "timer-predicate"))
(Inheritance (GroundedPredicate "scm: person_appears") (Predicate "see"))
(Inheritance (GroundedPredicate "scm: emotion") (Predicate "emotion"))
