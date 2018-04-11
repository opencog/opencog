(define-module (opencog ghost procedures)
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

    ; schemas
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

(load "procedures/predicates.scm")
(load "procedures/schemas.scm")

; --------------------------------------------------------------
; Utilities for the predicates & schemas
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
