;; Utilities for the inference control learning experiment

(use-modules (srfi srfi-1))
(use-modules (opencog logger))
(use-modules (opencog randgen))
(use-modules (opencog query))
(use-modules (opencog rule-engine))

;; Set a logger for the experiment
(define icl-logger (cog-new-logger))
(cog-logger-set-component! icl-logger "ICL")
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

;; Remove dangling atoms from an atomspace. That is atoms with default
;; TV (null confidence) with empty incoming set
(define (remove-dangling-atoms as)
  (let* ((old-as (cog-set-atomspace! as)))
    (remove-dangling-atom-list (get-all-atoms))
    (cog-set-atomspace! old-as)))

(define (remove-dangling-atom-list atoms)
  (for-each remove-dangling-atom atoms))

;; Remove the atom from the current atomspace if it is dangling. That
;; is it has an empty incoming set and its TV has null
;; confidence. Then call recursively on its outgoing set.
(define (remove-dangling-atom atom)
  (if (and (cog-atom? atom) (null-incoming-set? atom) (null-confidence? atom))
      (let* ((outgoings (cog-outgoing-set atom)))
        (cog-delete atom)
        (remove-dangling-atoms outgoings))))

(define (null-incoming-set? atom)
  (null? (cog-incoming-set atom)))

(define (null-confidence? atom)
  (= 0 (tv-conf (cog-tv atom))))

;; Copy all atoms from an atomspace to another atomspace
(define (cp-as src dst)
  (let ((old-as (cog-set-atomspace! src)))
    (cog-cp-all dst)
    (cog-set-atomspace! old-as)))

;; Get atoms of a certain types from a given atomspace
(define (cog-get-atoms-as as type)
  (let ((old-as (cog-set-atomspace! as))
        (atoms (cog-get-atoms type)))
    (cog-set-atomspace! old-as)
    atoms))

;; Clear a given atomspace
(define (clear-as as)
  (let ((old-as (cog-set-atomspace! as)))
    (clear)
    (cog-set-atomspace! old-as)))

;; Apply a rule to the given atoms
(define (apply-rule rule . atoms)
  ;; Create a temporary atomspace, Copy the rule and the atoms in it
  (let ((tmp-as (cog-new-atomspace)))
    (cog-cp (cons rule atoms) tmp-as)
    ;; Switch to the temporary atomspace and apply the rule
    (let ((init-as (cog-set-atomspace! tmp-as))
          (results (cog-outgoing-set (cog-bind rule))))
      ;; Switch back to the initial atomspace and return the results
      (cog-set-atomspace! init-as)
      results)))
