(define-module (opencog ghost predicates)
  #:use-module (opencog)
  #:use-module (opencog attention)
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
    )
)

; Perception and action apis and predicates
; NOTE: This is not organized  as it is still in development.

; --------------------------------------------------------------
; There is a spacemap in eva-model module called faces. Thus the rename.
(define facemap (SpaceMapNode "perceived-faces"))

; --------------------------------------------------------------
; Apis for inputing sensory information.
; --------------------------------------------------------------
(define (perceived-face face-id x y z)
  (cog-pointmem-map-atom facemap (Concept face-id)
    (List (Number x) (Number y) (Number z)))
)

(define (perceived-emotion face-id emotion-type strength)
  (Evaluation (stv strength 1)
    (Predicate emotion-type)
    (List face))
)

(define (perceive-word word)
  *uspecified* 
)

; --------------------------------------------------------------
; Apis for forming GroundedPredicates that are used for 
; checking if the world is in a particular state or not.
; --------------------------------------------------------------
(define (person_appears face-id)
  (cog-pointmem-get-locs-of-atom facemap (Concept face-id))
)

(define (person_smiles)
  *uspecified* 
)

(define (person_angry)
  *uspecified* 
)

(define (word_perceived)
  *uspecified* 
)

; --------------------------------------------------------------
; Apis for forming GroundedSchemas that are use for exectuing
; actions.
; --------------------------------------------------------------
(define (animation)
  *uspecified* 
)

(define (expression)
  *uspecified* 
)
