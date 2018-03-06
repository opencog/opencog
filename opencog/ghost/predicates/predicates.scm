(define-module (opencog ghost predicates)
  #:use-module (ice-9 optargs)
  #:use-module (opencog)
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

    ; Actions
    animation
    expression

    ; Utilities
    is-model-true?
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

(define (perceived-face face-id strength)
  (cog-set-tv! (see-face face-id) (stv strength 1))
)

(define (perceived-emotion face-id emotion-type strength)
  (cog-set-tv! (face-emotion face-id emotion-type) (stv strength 1))
)

(define (perceive-word word)
  *unspecified*
)

; --------------------------------------------------------------
; Apis for forming GroundedPredicates that are used for
; checking if the world is in a particular state or not.
; --------------------------------------------------------------
;(define (person_appears face-id)
;  (cog-pointmem-get-locs-of-atom facemap (Concept face-id))
;)

(define* (person_appears #:optional face-id)
  (if face-id
    (is-model-true? (see-face face-id))
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

(define* (person_smiles #:optional face-id)
  (if face-id
    (is-model-true? (face-emotion face-id "smile"))
    (any-person-emotion? "smile")
  )
)

(define* (person_angry #:optional face-id)
  (if face-id
    (is-model-true? (face-emotion face-id "angry"))
    (any-person-emotion? "angry")
  )
)

(define (word_perceived)
  *unspecified*
)

; --------------------------------------------------------------
; Apis for forming GroundedSchemas that are use for exectuing
; actions.
; --------------------------------------------------------------
(define (animation)
  *unspecified*
)

(define (expression)
  *unspecified*
)

; --------------------------------------------------------------
; Utilities
; --------------------------------------------------------------
(define (is-model-true? model)
  (let ((strength (tv-mean (cog-tv model))))
    (cond
      ((> 0.5 strength) (stv 0 1))
      ((<= 0.5 strength) (stv 1 1)))
  )
)
