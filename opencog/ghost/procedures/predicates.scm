; Perception APIs and predicates

; --------------------------------------------------------------
; APIs for forming GroundedPredicates that are used for
; checking if the world is in a particular state or not.
; --------------------------------------------------------------

(define* (any-check proc sign #:optional (proc-type #t))
"
  any-check PROC SIGN [PROC-TYPE]

  Check if any of the models that can be fetched by the SignatureLink SIGN
  returns (stv 1 1) when checked against the procedure PROC. The procedure
  type is assumed to be true-state related unless PROC-TYPE is set to '#f'
  during which the procedure is assumed to be related with false-state.
"
  ; NOTE: The proc-type is added to deal with the case when there hasn't
  ; been any models in the atomspace. In such a scenario every thing
  ; is assumed to be as false state, as we don't yet have a don't know state.
  (let ((models (get-models sign)))
    (cond
      ((and (not proc-type) (null? models)) (stv 1 1))
      ((any (lambda (x) (equal? (stv 1 1) (proc x))) models) (stv 1 1))
      (else (stv 0 1))
    )
  )
)

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
(define-syntax get-model
  (syntax-rules ()
    ((_  func-name face-id)
      (func-name (cog-name face-id)))
    ((_  func-name face-id face-feature-1)
      (func-name (cog-name face-id) (cog-name face-feature-1)))
  )
)

(define-syntax-rule
  (define-face-predicates (model-func face-feature-nodes ...)
    predicate-node signature-link
    t-transitioning? t-occuring? f-transitioning? f-occuring?)
  ; The definitons are not public so as to be able to control which
  ; of them are exported by this module.
  (begin
    (define* (t-transitioning? face-feature-nodes ... #:optional face-id)
      (if face-id
        (true-event-occuring?
          (get-model model-func face-id face-feature-nodes ...))
        (any-check true-event-occuring? signature-link)
      )
    )
    (Implication
      (GroundedPredicate (format #f "scm: ~a" 't-transitioning?))
       predicate-node)

    (define* (t-occuring? face-feature-nodes ... #:optional face-id)
      (if face-id
        (true-perception-occuring?
          (get-model model-func face-id face-feature-nodes ...))
        (any-check true-perception-occuring? signature-link)
      )
    )
    (Implication
      (GroundedPredicate (format #f "scm: ~a" 't-occuring?))
       predicate-node)

    ; FIXME: This has issues when the window of perception (dti) is passed.
    ; Just because we don't know it doesn't mean it is true
    ;(define* (since-t? secs #:optional (face-id any-node))
    ;  (since-event-started-occuring?
    ;    (get-model model-func face-id face-feature-nodes ...)))
    ;(Implication
    ;  (GroundedPredicate (format #f "scm: ~a" 'since-t?))
    ;   predicate-node)

    (define* (f-transitioning? face-feature-nodes ... #:optional face-id)
      (if face-id
        (false-event-occuring?
          (get-model model-func face-id face-feature-nodes ...))
        (any-check false-event-occuring? signature-link)
      )
    )
    (Implication
      (GroundedPredicate (format #f "scm: ~a" 'f-transitioning?))
       predicate-node)

    (define* (f-occuring? face-feature-nodes ... #:optional face-id)
      (define (not-occuring? faceid) (negate-stv! (t-occuring? faceid)))
      (if face-id
        (not-occuring? face-id)
        (any-check not-occuring? signature-link #f)
      )
    )
    (Implication
      (GroundedPredicate (format #f "scm: ~a" 'f-occuring?))
       predicate-node)

    ; FIXME: This has issues when the window of perception (dti) is passed.
    ; Just because we don't know it doesn't mean it is true
    ;(define* (since-f? secs  #:optional (face-id any-node))
    ;  (since-event-stopped-occuring?
    ;    (get-model model-func face-id face-feature-nodes ...)))
    ;(Implication
    ;  (GroundedPredicate (format #f "scm: ~a" 'since-f?))
    ;   predicate-node)
  )
)

; --------------------------------------------------------------
; Define predicates for face-talking
(define-face-predicates (face-talking)
   face-talking-predicate face-talking-sign
   new_talking
   talking
   end_talking
   not_talking
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

;(set-procedure-property! after_user_started_talking 'documentation
;"
;  after_user_started_talking SECS [FACE-ID]
;
;  Returns (stv 1 1) if current time >= the time that the user identified by
;  FACE-ID started talking + SECS. Otherwise, returns (stv 0 1).
;
;  IF FACE-ID is not passed then the return value is for any person.
;"
;)

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

;(set-procedure-property! after_user_stopped_talking 'documentation
;"
;  after_user_stopped_talking SECS [FACE-ID]
;
;  Returns (stv 1 1) if current time >= the time that the user identified by
;  FACE-ID stopped talking + SECS. Otherwise, returns (stv 0 1).
;
;  IF FACE-ID is not passed then the return value is for any person.
;"
;)

; --------------------------------------------------------------
; Define predicates for face-visiblity
(define-face-predicates (see-face) see-face-predicate see-face-sign
   new_face
   face
   end_face
   no_face
)

(set-procedure-property! face 'documentation
"
  face [FACE-ID]

  Check if face with FACE-ID was seen.

  IF FACE-ID is not passed then the return value is for any person.
"
)

; --------------------------------------------------------------
; Define predicates for emotions of faces
(define-face-predicates (face-emotion emotion-type)
   face-emotion-predicate face-emotion-sign
   new_emotion
   emotion
   end_emotion
   no_emotion
)

(set-procedure-property! emotion 'documentation
"
  emotion EMOTION-TYPE [FACE-ID]

  Check if face with FACE-ID was seen to have EMOTION-TYPE emotion.
  Returns (stv 1 1) if the the model associated with FACE-ID is true,
  within the default-time-interval, otherwise it returns (stv 0 1).

  IF FACE-ID is not passed then the return value is for any person.
"
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

; --------------------------------------------------------------
(define* (any_answer #:optional source)
"
  any_answer [SOURCE]

  Returns (stv 1 1) if SOURCE has a result and (stv 0 1) if not. If SOURCE
  is not passed then it will check if any of the sources have an answer, and
  cache a random source from the list.
"
  ;TODO: The assumption is that this is used by an ordered goal. Make it
  ; handle unordered goal.
  (define src-list (get-sources))
  (define sent
    (cog-value (if source source (car src-list)) source-latest-sent-key))

  (define (has-result? src)
    ; Sometimes a check maybe run before any query is made to the source.
    (if (null? sent)
      (stv 0 1)
      (source-has-result? sent src)))

  (define srcs-with-result '())
  (define (any-result?)
    (set! srcs-with-result
      (filter (lambda (x) (equal? (stv 1 1) (has-result? x))) src-list))
    (not (null? srcs-with-result)))
  (define (pick-src)
    (list-ref srcs-with-result
      (random (length srcs-with-result) (random-state-from-platform))))

  (cond
    (source (has-result? source))
    ((any-result?) (cog-set-value! sent (Predicate "random-source") (pick-src))
      (stv 1 1))
    ; If the sources haven't finished processing
    (else (stv 0 1))
  )
)

; --------------------------------------------------------------
(define (neck_dir dir)
"
  neck_dir DIR

  Check if the current neck direction is equal to DIR.
"
  (define current-dir (cog-name (get_neck_dir)))

  (if (string-ci=? (cog-name dir) current-dir)
    (stv 1 1)
    (stv 0 1)
  )
)

; --------------------------------------------------------------
; Create the GroundedPredicateNode, and link it to a generic "timer-predicate"
; so that we can stimulate the generic one and the STI will diffuse to
; the specific predicates connecting to it
; TODO: Replace the ConceptNode with a PredicateNode
(Implication (GroundedPredicate "scm: after_min") timer-predicate)
(Implication (GroundedPredicate "scm: emotion") (Predicate "emotion"))
