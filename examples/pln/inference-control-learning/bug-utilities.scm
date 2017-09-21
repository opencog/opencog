;; Utilities for the inference control learning experiment

(use-modules (opencog logger))
(use-modules (opencog query))
(use-modules (opencog rule-engine))

;; Log the given atomspace at some level
(define (cog-logger-info-atomspace as)
  (cog-logger-info "~a" (atomspace->string as)))
(define (cog-logger-debug-atomspace as)
  (cog-logger-debug "~a" (atomspace->string as)))

;; Convert the given atomspace into a string.
(define (atomspace->string as)
  (let* ((old-as (cog-set-atomspace! as))
         ;; Get all atoms in as
         (all-atoms (get-all-atoms))
         (all-atoms-strings (map atom->string (get-all-atoms)))
         (all-atoms-string (apply string-append all-atoms-strings)))
    (cog-set-atomspace! old-as)
    all-atoms-string))

(define (get-all-atoms)
  (apply append (map cog-get-atoms (cog-get-types))))

;; Convert the given atom into a string if its incoming set is null,
;; otherwise the string is empty.
(define (atom->string h)
  (if (null? (cog-incoming-set h))  ; Avoid redundant corrections
      (format #f "~a" h)
      ""))
