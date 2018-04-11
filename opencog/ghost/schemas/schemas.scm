(define-module (opencog ghost schemas)
  #:use-module (ice-9 optargs)
  #:use-module (opencog)
  #:use-module (opencog exec)
  #:export (
    animation
    expression
    start_timer
  )
)

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
;  (set-time-perceived! timer-id)
  fini
)
