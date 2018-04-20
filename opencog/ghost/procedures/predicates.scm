; Perception APIs and predicates

; --------------------------------------------------------------
; There is a spacemap in eva-model module called faces. Thus the rename.
;(define facemap (SpaceMapNode "perceived-faces"))

; --------------------------------------------------------------
; APIs used to create the atoms used to represent the world, aka
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
; APIs for inputing sensory information.
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
  (define wn (Word word))
  (set-time-perceived! wn)

  ; 'ghost-word-seq' is shared among the rules with word-related pattern
  ; This is mainly to make sure the rules with only a wildcard in the pattern
  ; will also get some non-zero STI.
  ; TODO: Find some better representation for that
  ; (ghost-stimulate ghost-word-seq)

  (ghost-stimulate wn)
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
