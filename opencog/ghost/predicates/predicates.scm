(define-module (opencog ghost predicates)
  #:use-module (ice-9 optargs)
  #:use-module (opencog)
  #:use-module (opencog atom-types)
  #:use-module (opencog attention)
  #:use-module (opencog exec)
  #:use-module (opencog pointmem)
  #:use-module (opencog spacetime)
  #:export (
    ; Sensory input
    perceived-face
    perceived-emotion
    perceive-word

    ; Perceptual predicates
    person_appears
    person_smiles
    person_angry
    word_perceived

    ; Time related predicates
    after_min

    ; Actions
    animation
    expression
    start_timer

    ; Utilities
    is-model-true?
    any-model-true?
    set-time-perceived! ; temporarily exported
    time-perceived ; temporarily exported
    was-perceived?
  )
)

; Perception and action apis and predicates
; NOTE: This is not organized  as it is still in development.

; --------------------------------------------------------------
; There is a spacemap in eva-model module called faces. Thus the rename.
;(define facemap (SpaceMapNode "perceived-faces"))

; --------------------------------------------------------------
; Apis used to create the atoms used to represent the world, aka
; world-model. These are not exported.
; --------------------------------------------------------------
(define (see-face face-id)
"
  Define the atom used to represent that the face represented by FACE-ID
  is being seen.
"
  (Evaluation
    (Predicate "see")
    (List
      (Concept "I")
      (Concept face-id)))
)

(define (face-emotion face-id emotion-type)
"
  Define the atom used to represent EMOTION-TYPE of the face with
  id FACE-ID.
"
  (Evaluation
    (Predicate emotion-type)
    (List
      (Concept face-id)))
)

; --------------------------------------------------------------
; Apis for inputing sensory information.
; --------------------------------------------------------------
;(define (perceived-face face-id x y z)
;  (cog-pointmem-map-atom facemap (Concept face-id)
;    (List (Number x) (Number y) (Number z)))

(define (perceived-face face-id confidence)
  (cog-set-tv! (see-face face-id) (stv 1 confidence))
)

(define (perceived-emotion face-id emotion-type confidence)
  (cog-set-tv! (face-emotion face-id emotion-type) (stv 1 confidence))
)

(define (perceive-word word)
  (set-time-perceived! (Word word))
)

; --------------------------------------------------------------
; APIs for forming GroundedPredicates that are used for
; checking if the world is in a particular state or not.
; --------------------------------------------------------------
;(define (person_appears face-id)
;  (cog-pointmem-get-locs-of-atom facemap (Concept face-id))
;)

(define* (person_appears #:optional face-id)
  (if face-id
    (is-model-true? (see-face (cog-name face-id)))
    (any-face-seen?)
  )
)

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
      (any-model-true? models)
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
      (any-model-true? models)
    )
  )
)

; TODO: If the stream of sensory inputs are interupted, for whatever reason,
; then the variations in the confidence value are not updated and thus the
; state of the world wouldn't be correct. To fix this add a time window
; similar to word_perceived. If the time-window is passed then it returns
; false.
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

(define* (word_perceived word #:optional time-interval)
  (if time-interval
    (was-perceived? word (current-time-us)
      (string->number (cog-name time-interval)))
    (was-perceived? word (current-time-us) 0.01)
  )
)

; --------------------------------------------------------------
; Apis for forming GroundedSchemas that are use for exectuing
; actions.
; NOTE: For testing use (opencog eva-behavior) module. For
; running use (opencog movement) module. This is because the
; apis are atomese DefinedPredicates.
; TODO: List out the DefinedPredicates that are used as api, so as to
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

; --------------------------------------------------------------
; Utilities
; --------------------------------------------------------------
(define (is-model-true? model)
; It is better to find the product of the strength and confidence but for now
; confidence is used as the default stv for new atoms is (stv 1 0), so
; need to waste cpu cycles.
  (let ((confidence (tv-conf (cog-tv model))))
    (cond
      ((> 0.5 confidence) (stv 0 1))
      ((<= 0.5 confidence) (stv 1 1)))
  )
)

(define (any-model-true? model-list)
  (let ((true-models (filter is-model-true? model-list)))
    (if (null? true-models)
      (stv 0 1)
      (stv 1 1)
    )
  )
)

; --------------------------------------------------------------
; These allow adding time based predicates.
(define time-key (Predicate "time-perceived"))

(define (current-time-us)
"
  Returns the current-time including microseconds by converting the pair
  returned by (gettimeofday) into a floating number representing seconds
  with microsecond accuracy.
"
  (let ((time (gettimeofday)))
    (+ (car time) (/ (cdr time) 1000000))
  )
)

(define (set-time-perceived! atom)
"
  set-time-perceived! ATOM

  Record the current time as the time of perception of ATOM, and return ATOM.
"
  (cog-set-value! atom time-key (FloatValue (current-time-us)))
)

(define (time-perceived atom)
"
  time-perceived ATOM

  Return the time of perception of the ATOM, or nil. Nil is used to denote
  that there is no perception time associated with ATOM.
"
  ; If a perception time was not set then it is assumed to have not been
  ; perceived and nil is returned.
  (let ((time (cog-value atom time-key)))
    (if (null? time)
      time
      (cog-value-ref time 0)
    )
  )
)

(define (time-within-interval? time end-time time-interval)
"
  time-within-interval? TIME END-TIME TIME-INTERVAL

  Returns #t if (END-TIME - TIME-INTERVAL) <= TIME <= END-TIME, else it
  returns #f. All times passed as argument should be in seconds.
"
  (and
    (<= (- end-time time-interval) time)
    (<= time end-time))
)

(define (was-perceived? atom end-time time-interval)
"
  was-perceived? ATOM END-TIME TIME-INTERVAL

  Returns (stv 1 1) if
    (END-TIME - TIME-INTERVAL) <= time-perceived <= END-TIME, else it
  returns (stv 0 1). All times passed as argument should be in seconds.
"
  (let ((time (time-perceived atom)))
    (if (and (not (null? time))
          (time-within-interval? time end-time time-interval))
      (stv 1 1)
      (stv 0 1)
    )
  )
)

; This is an action, that can be wrapped in a GroundedSchemaNode
(define* (start_timer #:optional (timer-id (Concept "Default-Timer")))
"
  start_timer TIMER-ID (optional)

  Record the current time for TIMER-ID.
  If TIMER-ID is not given, a default timer will be used.
"
  (Member timer-id (Concept "Timer"))
  (set-time-perceived! timer-id)
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
