;; =======================================================================
;; Implication Instantiation rule
;; (TODO add wiki page)
;;
;; ImplicationLink
;;    V
;;    P
;;    Q
;; T
;; |-
;; Q[V->T]
;;
;; where V is a variable or a list of variables, P is a condition, Q
;; is the implicand, T is an atom (or a list of atoms) to substitute
;; and Q[V->T] is Q where V has been substituted by T.
;;
;; As currently implemented T is not explicitely in the
;; premises. Instead it is queried directly by the rule's formula.
;; -----------------------------------------------------------------------

(use-modules (srfi srfi-1))

(use-modules (opencog exec))
(use-modules (opencog logger))

(load "instantiation.scm")

;;;;;;;;;;;;;;;;;;;;;;;
;; Helper definition ;;
;;;;;;;;;;;;;;;;;;;;;;;

(define implication-full-instantiation-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-instantiation-body
  (ImplicationLink
     (VariableNode "$TyVs")
     (VariableNode "$P")
     (VariableNode "$Q")))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Implication full instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define implication-full-instantiation-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-full-instantiation-formula")
     (ListLink
        (VariableNode "$TyVs")
        (VariableNode "$P")
        (VariableNode "$Q"))))

;; Only try to match an ImplicationLink with a type restricted
;; variable in the ImplicationLink variable definition. The choice of
;; the substitution term is done randomly in
;; implication-full-instantiation-formula. All scoped variables are
;; instantiated.
(define implication-full-instantiation-rule
  (BindLink
     implication-full-instantiation-variables
     implication-instantiation-body
     implication-full-instantiation-rewrite))

;; This function
;;
;; 1. randomly selects a substitution term (or a tuple of substitution
;;    terms, if the ImplicationLink has multiple variables in scope)
;;    that meets the implication's condition (the implicant),
;;
;; 2. performs the substitution,
;;
;; 3. calculates its TV (TODO: just <1 1> for now).
(define (implication-full-instantiation-formula SV P Q)
  (cog-set-tv!
   (let ((terms (select-conditioned-substitution-terms SV P)))
     ;; Substitute the variable by the term in the body
     (cog-execute! (PutLink (LambdaLink SV Q) terms)))
   (stv 1 1)))

;; Name the rule
(define implication-full-instantiation-rule-name
  (Node "implication-full-instantiation-rule"))
(DefineLink implication-full-instantiation-rule-name
  implication-full-instantiation-rule)

;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ;; Implication partial instantiation rule ;;
;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (define implication-partial-instantiation-variables
;;   (VariableList
;;      (TypedVariableLink
;;         (VariableNode "$TyVs")
;;         (TypeNode "VariableList"))
;;      (VariableNode "$P")
;;      (VariableNode "$Q")))

;; (define implication-partial-instantiation-rewrite
;;   (ExecutionOutputLink
;;      (GroundedSchemaNode "scm: implication-partial-instantiation-formula")
;;      (ListLink
;;         (VariableNode "$TyVs")
;;         (VariableNode "$P")
;;         (VariableNode "$Q"))))

;; ;; Like implication-full-instantiation-rule but only instantiate one
;; ;; variable amonst a variable list (if there is just one variable in
;; ;; the implication scope, then this rule will not be invoked).
;; (define implication-partial-instantiation-rule
;;   (BindLink
;;      implication-partial-instantiation-variables
;;      implication-instantiation-body
;;      implication-partial-instantiation-rewrite))

;; ;; This function
;; ;;
;; ;; 1. randomly selects a substitution term that meets the
;; ;;    implication's condition (the implicant),
;; ;;
;; ;; 2. performs the substitution,
;; ;;
;; ;; 3. calculates its TV (TODO: just <1 1> for now).
;; (define (implication-partial-instantiation-formula TyVs P Q)
;;   (cog-set-tv!
;;    (let* (
;;           (TyV-and-remain (select-rm-rnd-outgoing TyVs))
;;           (TyV (car TyV-and-remain))
;;           (TyVs-remain (cadr TyV-and-remain))
;;           (TyVs-remain-len (length (cog-outgoing-set TyVs-remain)))
;;                                         ; Select the term to substitute
;;           (term (select-substitution-term TyV TODO: use P))
;;                                         ; Substitute the variable by
;;                                         ; the term in the body
;;           (Q-inst (cog-execute! (PutLink (LambdaLink TyV Q) term)))
;;                                         ; If there is only one
;;                                         ; variable left, discard the
;;                                         ; TypedVariableLink
;;           (TyVs-remain (if (= TyVs-remain-len 1)
;;                            (gar TyVs-remain)
;;                            TyVs-remain)))
;;      (if (> TyVs-remain-len 0)
;;                                         ; If there are some variables
;;                                         ; left, add the ImplicationLink
;;                                         ; back
;;          (ImplicationLink TyVs-remain Q-inst)
;;                                         ; Otherwise just return the
;;                                         ; Q instance
;;          Q-inst))
;;    ;; TODO: implement a probabilistic formula rather than <1 1>
;;    (stv 1 1)))

;; ;; Name the rule
;; (define implication-partial-instantiation-rule-name
;;   (Node "implication-partial-instantiation-rule"))
;; (DefineLink implication-partial-instantiation-rule-name
;;   implication-partial-instantiation-rule)
