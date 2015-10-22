;; Helpers for instantiation rules like forall full and partial
;; instantiation rules and implication full and partial instantiation
;; rules.

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

;; Select a random atom from a Link's outgoings
(define (select-rnd-outgoing link)
  (let ((outgoings (cog-outgoing-set link)))
    (list-ref outgoings (random (length outgoings)))))

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
                                        ; Select one randomly
    (select-rnd-outgoing result)))
