;; Utilities for the inference control learning experiment

(use-modules (srfi srfi-1))
(use-modules (opencog logger))
(use-modules (opencog randgen))

;; Set a logger for the experiment
(define icl-logger (cog-new-logger))
(cog-logger-set-component! icl-logger "ICL")
(cog-logger-set-stdout! icl-logger #t)
;; (cog-logger-set-sync! icl-logger #t)
(define (icl-logger-error . args) (apply cog-logger-error (cons icl-logger args)))
(define (icl-logger-warn . args) (apply cog-logger-warn (cons icl-logger args)))
(define (icl-logger-info . args) (apply cog-logger-info (cons icl-logger args)))
(define (icl-logger-debug . args) (apply cog-logger-debug (cons icl-logger args)))
(define (icl-logger-fine . args) (apply cog-logger-fine (cons icl-logger args)))

;; Let of characters of the alphabet
(define alphabet-list
  (string->list "ABCDEFGHIJKLMNOPQRSTUVWXYZ"))

;; Given a number between 0 and 25 return the corresponding letter as
;; a string.
(define (alphabet-ref i)
  (list->string (list (list-ref alphabet-list i))))

;; Randomly select between 2 ordered letters and create a target
;;
;; Inheritance
;;   X
;;   Y
(define (gen-random-target)
  (let* ((Ai (cog-randgen-randint 25))
         (Bi (+ Ai (random (- 26 Ai))))
         (A (alphabet-ref Ai))
         (B (alphabet-ref Bi)))
    (Inheritance (Concept A) (Concept B))))

;; Randomly generate N targets
(define (gen-random-targets N)
  (if (= N 0)
      '()
      (cons (gen-random-target) (gen-random-targets (- N 1)))))

;; Log the given atomspace at some level
(define (icl-logger-info-atomspace as)
  (icl-logger-info "~a" (atomspace->string as)))
(define (icl-logger-debug-atomspace as)
  (icl-logger-debug "~a" (atomspace->string as)))

;; Convert the given atomspace into a string.
(define (atomspace->string as)
  (let* ((old-as (cog-set-atomspace! as))
         ;; Get all atoms in as
         (all-atoms (apply append (map cog-get-atoms (cog-get-types))))
         (atom->string (lambda (h)
                         (if (null? (cog-incoming-set h))  ; Avoid redundant
                                                           ; corrections
                             (format "~a" h)
                             "")))
         (all-atoms-string (apply string-append (map atom->string all-atoms))))
    (cog-set-atomspace! old-as)
    all-atoms-string))
