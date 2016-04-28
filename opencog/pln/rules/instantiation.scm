;; Helpers for instantiation rules like forall full and partial
;; instantiation rules and implication full and partial instantiation
;; rules.

(use-modules (srfi srfi-1))
(use-modules (opencog logger))

;; Given an atom
;;
;; (TypedVariableLink
;;    (VariableNode "$V")
;;    <type-constraint>)
;;
;; return a random term satisfying the type constraint.
(define (select-substitution-term TyV)
  (let* (
         (V (gar TyV))
                                        ; Build pattern matcher query
                                        ; for the subtitution term
         (query (GetLink TyV V))
                                        ; Fetch all possible substitution terms
         (result (cog-execute! query)))
    ;; Select one randomly, but first purge the query to not pollute
    ;; the atomspace
    (extract-hypergraph query)
    (select-rnd-outgoing result)))

;; Given an atom
;;
;; (VariableList
;;    (TypedVariableLink
;;       (VariableNode "$V1")
;;       <type-constraint1>)
;;    ...
;;    (TypedVariableLink
;;       (VariableNode "$Vn")
;;       <type-constraintn>))
;;
;; return a ListLink of random terms satisfying the type constraints of
;; V1 to Vn.
(define (select-substitution-terms TyVs)
  (let* (
                                        ; Get the list of typed
                                        ; variables V1 to Vn
         (typed-vars (cog-outgoing-set TyVs))
                                        ; Pick up a random term of each
         (terms (map select-substitution-term typed-vars)))
    (apply ListLink terms)))

;; Select a random atom from a Link's outgoings. If the link is empty
;; return the undefined handle.
(define (select-rnd-outgoing link)
  (let* ((outgoings (cog-outgoing-set link))
         (outgoings-len (length outgoings)))
    (if (= outgoings-len 0)
        (cog-undefined-handle)
        (list-ref outgoings (random outgoings-len)))))

;; Return a list without the indexed element
(define (rm-list-ref l i)
  (append (take l i) (drop l (+ i 1))))

;; Select a random atom from a Link's outgoings, return a pair composed of
;; 1. the selected atom
;; 2. a new link with the remaining outgoings
(define (select-rm-rnd-outgoing link)
  (let* ((link-type (cog-type link))
         (outgoings (cog-outgoing-set link))
         (rnd-index (random (length outgoings)))
         (rnd-atom (list-ref outgoings rnd-index))
         (remain-list (rm-list-ref outgoings rnd-index))
         (remain (apply cog-new-link link-type remain-list)))
    (list rnd-atom remain)))

;; Given a VariableNode V and atom A, check that V is free in A.
(define (is-free-variable-in-atom V A)
  (if (equal? V A)
      #t
      (any (lambda (c) (is-free-variable-in-atom V c)) (cog-outgoing-set A))))

;; Given a TypedVariable V and atom A, check that V is free in A.
(define (is-free-typed-variable-in-atom TyV A)
  (is-free-variable-in-atom (gar TyV) A))

;; Given a variable or a variable list, check that all variables are
;; free in A.
(define (is-all-typed-variables-free-in-atom TyVs A)
  (if (equal? (cog-type TyVs) 'VariableList)
      (every (lambda (TyV) (is-free-typed-variable-in-atom TyV A))
             (cog-outgoing-set TyVs))
      (is-free-typed-variable-in-atom TyVs A)))

;; Given 2 atoms, a list of variables (or a single variable) and a
;; condition P, return a random term list (or a single term in case
;; there was just a single variable) satisfying the type constraint
;; and the condition.
(define (select-conditioned-substitution-terms TyVs P)
  (if (is-all-typed-variables-free-in-atom TyVs P)
      (let* (
                                        ; Build pattern matcher query
                                        ; for the subtitution terms
             (query (GetLink TyVs P))
                                        ; Fetch all possible substitution terms
             (result (cog-execute! query)))
        ;; Select one randomly, but first purge the query to not pollute
        ;; the atomspace
        (extract-hypergraph query)
        (select-rnd-outgoing result))
      (cog-undefined-handle)))
