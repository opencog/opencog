;; Helpers for instantiation rules like forall full and partial
;; instantiation rules and implication full and partial instantiation
;; rules.

(use-modules (srfi srfi-1))
(use-modules (opencog logger))

;; Given a variable, typed or not, return the corresponding untyped
;; variable, that is where its type signature has been stripped of it.
(define (strip-type-from-variable TyV)
  (if (equal? (cog-type TyV) 'TypedVariableLink)
      (gar TyV)
      (TyV)))

;; Given an atom
;;
;; (TypedVariableLink
;;    (VariableNode "$V")
;;    <type-constraint>)
;;
;; return a random term satisfying the type constraint.
(define (select-substitution-term TyV)
  (let* (
         (V (strip-type-from-variable TyV))
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
(define (variable-free? V A)
  (if (member V (cog-free-variables A)) #t #f))

;; Given a TypedVariable V and atom A, check that V is free in A.
(define (typed-variable-free? TyV A)
  (variable-free? (strip-type-from-variable TyV) A))

;; Given a Variable or a VariableList return a list of the variable(s)
(define (variable-list TyVs)
  (if (equal? (cog-type TyVs) 'VariableList)
      (cog-outgoing-set TyVs)
      (list TyVs)))

;; Given a variable or a variable list, check that all variables are
;; free in A.
(define (typed-variables-free? TyVs A)
  (every (lambda (TyV) (typed-variable-free? TyV A)) (variable-list TyVs)))

;; Given a typed variable or typed variable list, return a
;; VariableList of all typed variables free in A.
(define (filter-free-typed-variables TyVs A)
  (let ((typed-variable-free-in-A? (lambda (TyV) (typed-variable-free? TyV A))))
    (VariableList (filter typed-variable-free-in-A? (variable-list TyVs)))))

;; Given a typed variable or typed variable list, return a list of all
;; variables not free in A, stripped from their types.
(define (unfree-variables TyVs A)
  (let* ((unfree-var-in-A? (lambda (TyV) (not (typed-variable-free? TyV A))))
         (unfree-typed-vars (filter unfree-var-in-A? (variable-list TyVs)))
         (unfree-vars (map strip-type-from-variable unfree-typed-vars)))
    unfree-vars))

;; Given 2 atoms, a list of variables (or a single variable) and a
;; condition P, return a random term list (or a single term in case
;; there was just a single variable) satisfying the type constraint
;; and the condition. Works even if TyVs has extra variables not in P,
;; in such a case the corresponding terms only need to satisfy the
;; type contraints.
(define (select-conditioned-substitution-terms TyVs P)
  (let* (
         ;; Get all unfree variables stripped from their types
         (unfree-vars (unfree-variables TyVs P))
         ;; Build pattern matcher query for the subtitution terms
         (query-body (if (null? unfree-vars) P (And P unfree-vars)))
         (query (GetLink TyVs query-body))
         ;; Fetch all possible substitution terms
         (results (cog-execute! query)))
        ;; Select one randomly, but first purge the query to not
        ;; pollute the atomspace
        (extract-hypergraph query)
        (select-rnd-outgoing results)))
