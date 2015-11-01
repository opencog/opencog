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
        ;; (ImplicationLink
           (VariableNode "$TyVs")
           (VariableNode "$P")
           (VariableNode "$Q"))));;)

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
;; 2. performs the substitution.
;;
;; 3. calculates its TV (TODO: just <1 1> for now, but might be just
;;    the TV on the ImplicationLink)
;;
;; If no substitution is possible it returns the undefined handle
(define (implication-full-instantiation-formula SV P Q);;Impl)
  (let* (
         ;; (SV (gar Impl))
         ;; (P (gadr Impl))
         ;; (Q (gaddr Impl))
         (terms (select-conditioned-substitution-terms SV P)))
    (if (equal? terms (cog-undefined-handle))
        terms
        ;; Substitute the variables by the terms in the body
        (cog-set-tv!
         (cog-execute! (PutLink (LambdaLink SV Q) terms))
         (stv 1 1)))))
         ;; (cog-tv Impl)))))

;; Name the rule
(define implication-full-instantiation-rule-name
  (Node "implication-full-instantiation-rule"))
(DefineLink implication-full-instantiation-rule-name
  implication-full-instantiation-rule)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Implication partial instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define implication-partial-instantiation-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeNode "VariableList"))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-partial-instantiation-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-partial-instantiation-formula")
     (ListLink
        (VariableNode "$TyVs")
        (VariableNode "$P")
        (VariableNode "$Q"))))

;; Like implication-full-instantiation-rule but only instantiate one
;; variable amonst a variable list (if there is just one variable in
;; the implication scope, then this rule will not be invoked).
(define implication-partial-instantiation-rule
  (BindLink
     implication-partial-instantiation-variables
     implication-instantiation-body
     implication-partial-instantiation-rewrite))

;; This function
;;
;; 1. randomly selects a substitution term that partially meets the
;;    implication's condition (the implicant),
;;
;; 2. performs the substitution,
;;
;; 3. calculates its TV (TODO: just <1 1> for now).
;;
;; TODO: To make this function better a form of partial pattern
;; matching should be supported. Probably enabling self grounding in
;; the pattern matcher would do the trick (see
;; PatternMatchEngine::self_compare)
(define (implication-partial-instantiation-formula TyVs P Q)
  (let* (
         (TyVs-outgoings (cog-outgoing-set TyVs))
         (TyVs-outgoings-len (length TyVs-outgoings))
                                        ; Select all potential
                                        ; substitution terms
         (terms (select-conditioned-substitution-terms TyVs P))
                                        ; Choose the index of the
                                        ; variable to substitute
         (rnd-index (random TyVs-outgoings-len))
                                        ; Take it's corresponding
                                        ; variable
         (TyV (list-ref TyVs-outgoings rnd-index))
                                        ; Build a VariableList of the
                                        ; remaining variables
         (TyVs-remain-list (rm-list-ref TyVs-outgoings rnd-index))
         (TyVs-remain-len (length TyVs-remain-list))
         (TyVs-remain (apply cog-new-link 'VariableList TyVs-remain-list)))
    (if (equal? terms (cog-undefined-handle))
        terms
        (cog-set-tv!
         (let* (
                                        ; Take the corresponding term
                (term (list-ref (cog-outgoing-set terms) rnd-index))
                                        ; Substitute the variable by
                                        ; the term in the P and Q bodies
                (P-inst (cog-execute! (PutLink (LambdaLink TyV P) term)))
                (Q-inst (cog-execute! (PutLink (LambdaLink TyV Q) term)))
                                        ; If there is only one
                                        ; variable left, discard the
                                        ; VariableLink
                (TyVs-remain (if (= TyVs-remain-len 1)
                                 (gar TyVs-remain)
                                 TyVs-remain)))
           (if (> TyVs-remain-len 0)
                                        ; If there are some variables
                                        ; left, rebuild the
                                        ; ImplicationLink with the
                                        ; remaining variables
               (ImplicationLink TyVs-remain P-inst Q-inst)
                                        ; Otherwise just return the
                                        ; Q instance
               Q-inst))
         ;; TODO: implement a probabilistic formula rather than <1 1>
         (stv 1 1)))))

;; Name the rule
(define implication-partial-instantiation-rule-name
  (Node "implication-partial-instantiation-rule"))
(DefineLink implication-partial-instantiation-rule-name
  implication-partial-instantiation-rule)
