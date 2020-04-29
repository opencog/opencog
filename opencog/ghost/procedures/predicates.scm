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
      ((and (not proc-type) (nil? models)) (stv 1 1))
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

(define-syntax define-face-predicates (lambda (x)
  (define (new-id id suffix)
  "prefix = a string
   id = a pattern variable(aka keyword, template-identifier).
   See https://scheme.com/tspl4/syntax.html#./syntax:h4 for details.
  "
    (datum->syntax id (symbol-append (syntax->datum id) suffix)))

  (syntax-case x ()
    ; 'func-name' is the name of the predicate function.
    ((_ (model-func face-feature-nodes ...)
        predicate-node signature-link func-name)
      (with-syntax ((new (new-id #'func-name '_new))
                    (current (new-id #'func-name '_current))
                    (end (new-id #'func-name '_end)))
        #'(begin
          ; NOTE: 'face-feature-nodes ...' will match 0 or more arguments that
          ; 'func-name' accept.
          (define* (new face-feature-nodes ... #:optional face-id)
          ; Return (stv 1 1) if the face identified by FACE-ID is starting to
          ; show the state that is identified by face-feature-node, within
          ; the last event-period, otherwise returns (stv 0 1).
          ; IF FACE-ID is not passed then the return value is for any person.
            (if face-id
              (true-event-occuring?
                (get-model model-func face-id face-feature-nodes ...))
              (any-check true-event-occuring? signature-link)
            )
          )

          (define* (current face-feature-nodes ... #:optional face-id)
          ; Check if face with FACE-ID is showing the state that is identified
          ; by face-feature-node.
          ; IF FACE-ID is not passed then the return value is for any person.
            (if face-id
              (true-perception-occuring?
                (get-model model-func face-id face-feature-nodes ...))
              (any-check true-perception-occuring? signature-link)
            )
          )

          (define* (end face-feature-nodes ... #:optional face-id)
          ; Return (stv 1 1) if the face identified by FACE-ID is stopping
          ; showing the state that is identified by face-feature-node,
          ; within the last event-period, otherwise returns (stv 0 1).
          ; IF FACE-ID is not passed then the return value is for any person.
            (if face-id
              (false-event-occuring?
                (get-model model-func face-id face-feature-nodes ...))
              (any-check false-event-occuring? signature-link)
            )
          )

          (define* (func-name state face-feature-nodes ... #:optional face-id)
            (cond
              ((equal? "new" (cog-name state))
                 (new face-feature-nodes ... face-id))
              ((equal? "current" (cog-name state))
                 (current face-feature-nodes ... face-id))
              ((equal? "end" (cog-name state))
                 (end face-feature-nodes ... face-id)))
          )
          ; NOTE: When the predicate-node is stimulated then rules that have
          ; the GroundedPredicate will get stimulated during propogation of
          ; the stimulation.
          (Implication
            (GroundedPredicate (format #f "scm: ~a" 'func-name))
             predicate-node)

          )))

    ; 'func-name' is the name of the predicate function.
    ; 'negated-func-name' is the name of the negated predicate of 'func-name'
    ((_ (model-func face-feature-nodes ...)
        predicate-node signature-link func-name negated-func-name)
      #'(begin
        (define-face-predicates (mode-func face-feature-nodes ...)
          predicate-node signature-link func-name)

        (define* (negated-func-name face-feature-nodes ... #:optional face-id)
        ; Check if face with FACE-ID is not showing the state that is
        ; identified by face-feature-node.
        ; IF FACE-ID is not passed then the return value is for any person.
          (define (not-occuring? faceid) (negate-stv! ('current faceid)))
            (if face-id
              (not-occuring? face-id)
              (any-check not-occuring? signature-link #f)
            )
        )
        (Implication
          (GroundedPredicate (format #f "scm: ~a" 'negated-func-name))
          predicate-node)

        ))
  )))

; --------------------------------------------------------------
; Define predicates for face-talking
(define-face-predicates (face-talking)
  face-talking-predicate face-talking-sign is_talking is_not_talking
)

; --------------------------------------------------------------
; Define predicates for face-visiblity
(define-face-predicates (see-face)
  see-face-predicate see-face-sign is_face_perceived
)

; --------------------------------------------------------------
; Define predicates for emotions of faces
(define-face-predicates (face-emotion emotion-type)
   face-emotion-predicate face-emotion-sign is_emotion
)

; --------------------------------------------------------------
(define* (is_word_perceived word #:optional (time-interval dti-node))
  (was-perceived? (Word (cog-name word)) time-interval)
)

(define* (is_after_min minutes #:optional (timer-id (Concept "Default-Timer")))
"
  is_after_min MINUTES TIMER-ID (optional)

  Returns (stv 1 1) if current time >= the timer's start time (if given) + MINUTES.
  Otherwise, returns (stv 0 1)
"
  (define t (time-perceived timer-id))

  ; If it's null, the timer probably has not started yet
  (if (nil? t)
    (stv 0 1)
    (if (>= (current-time-us)
            (+ t (* (string->number (cog-name minutes)) 60)))
        (stv 1 1)
        (stv 0 1)))
)

; --------------------------------------------------------------
(define* (is_answer_received #:optional source)
"
  is_answer_received [SOURCE]

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
    (if (nil? sent)
      (stv 0 1)
      (source-has-result? sent src)))

  (define srcs-with-result '())
  (define (any-result?)
    (set! srcs-with-result
      (filter (lambda (x) (equal? (stv 1 1) (has-result? x))) src-list))
    (not (nil? srcs-with-result)))
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
(define (any_stochastic_question)
"
  any_stochastic_question

  Check if there is any stochastic question generated.
  Returns (stv 1 1) if so, (stv 0 1) otherwise.
"
  (if (nil? (cog-value (ghost-get-curr-sent) (Concept "StochasticQuestion")))
    (stv 0 1)
    (stv 1 1))
)

; --------------------------------------------------------------
(define (is_neck_direction dir)
"
  is_neck_direction DIR

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
(Implication (GroundedPredicate "scm: is_after_min") timer-predicate)
(Implication (GroundedPredicate "scm: emotion") (Predicate "emotion"))
