;; Utilities for the inference control learning experiment

(use-modules (srfi srfi-1))
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

;; Remove dangling atoms from an atomspace. That is atoms with default
;; TV (null confidence) with empty incoming set
(define (remove-dangling-atoms as)
  (let* ((old-as (cog-set-atomspace! as)))
    (for-each remove-dangling-atom (get-all-atoms))
    (cog-set-atomspace! old-as)))

;; Remove the atom from the current atomspace if it is dangling. That
;; is it has an empty incoming set and its TV has null confidence.
(define (remove-dangling-atom atom)
  (if (and (cog-atom? atom) (null-incoming-set? atom) (null-confidence? atom))
      (extract-hypergraph atom)))

(define (null-incoming-set? atom)
  (null? (cog-incoming-set atom)))

(define (null-confidence? atom)
  (= 0 (tv-conf (cog-tv atom))))

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
