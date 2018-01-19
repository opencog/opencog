;; Axiom rule about the top abstraction (Lambda X X)
;;
;; <texts>
;; <ms>
;; Evaluation <tv>
;;   GroundedPredicate "scm: size-ge"
;;     <texts>
;;     <ms>
;; |-
;; Evaluation <tv>
;;   Predicate "minsup"
;;   List
;;     <top>
;;     <texts>
;;     <ms>
;;
;; where <top> is (Lamdba X X), <texts> is a ConceptNode such that all
;; its members are the texts in consideration, and <ms> is the minimum
;; frequency in consideration.
;;
;; The grounded predicate size-ge checks whether the size of <texts>
;; is greater than or equal to <ms>.
;;
;; Basically <tv> will be (stv 1 1) if the number of members of
;; <texts> is greater than or equal to <ms>, (stv 0 1) otherwise.

(use-modules (opencog logger))
(use-modules (opencog query))
(use-modules (opencog rule-engine))

(cog-logger-set-level! "debug")
(cog-logger-set-stdout! #t)

(load "pattern-miner-utils.scm")

;; Note that due to the texts-ge-ms precondition, it will not be able
;; to proof its contrary, i.e. <tv> will never be assigned (stv 0 1).
;; To remedy that we may move texts-gt-ms in the formula.
(define top-abstraction-rule
  (let* (;; Variables
         (texts (Variable "$texts"))
         (ms (Variable "$ms"))
         (X (Variable "$X"))
         ;; Constants
         (top (Lambda X X))
         (minsup (Predicate "minsup"))
         ;; Types
         (NumberT (Type "NumberNode"))
         (ConceptT (Type "ConceptNode"))
         ;; Vardecls
         (texts-decl (TypedVariable texts ConceptT))
         (ms-decl (TypedVariable ms NumberT))
         (vardecl (VariableList texts-decl ms-decl))
         ;; Clauses
         (size-ge (GroundedPredicate "scm: size-ge"))
         (texts-ge-ms (Evaluation size-ge (List texts ms)))
         (clauses (And
                    texts
                    ms
                    texts-ge-ms))
         ;; Rewrite
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: top-abstraction-formula")
                    (List
                      (Evaluation
                        minsup
                        (List
                          top
                          texts
                          ms))
                      texts
                      ms))))
    (Bind
      vardecl
      clauses
      rewrite)))

(define (top-abstraction-formula conclusion . premises)
  (if (= (length premises) 2)
      (let* ((texts (car premises))
             (ms (cadr premises))
             (conclusion-tv (size-ge texts ms)))
        (if conclusion-tv
            (cog-set-tv! conclusion conclusion-tv)))))


