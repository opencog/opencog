;; Rule to generate shallow abstractions of a given pattern,
;; specifically
;;
;; Evaluation
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       VariableList
;;         <x1>
;;         ...
;;         <xn>
;;       <g-body>
;;     <texts>
;;     <ms>
;; |-
;; Set
;;   Evaluation (stv 1 1)
;;     Predicate "shallow-abstraction"
;;     List
;;       List
;;         <f1>
;;         <x2>
;;         ...
;;         <xn>
;;       Evaluation
;;         Predicate "minsup"
;;         List
;;           Lambda
;;             VariableList
;;               <x1>
;;               ...
;;               <xn>
;;             <g-body>
;;           <texts>
;;           <ms>
;; ...
;;
;; where f1 to fn are a shallow abstractions (shabs stands for shallow
;; abstraction) either functions, constant nodes, or a variable nodes
;; amongst x1 to xn.
;;
;; Note, it's weird that the second argument of shabs is the minsup
;; evaluation because it might be equivalent to it's TV, it's not
;; clear though that it's much of a problem, so we'll go with that for
;; now.

(load "miner-rule-utils.scm")

(define shallow-abstraction-rule
  (let* (;; Variables
         (g (Variable "$g"))
         (texts (Variable "$texts"))
         (ms (Variable "$ms"))
         ;; Types
         (LambdaT (Type "LambdaLink"))
         (PutT (Type "PutLink"))
         (ConceptT (Type "ConceptNode"))
         (NumberT (Type "NumberNode"))
         ;; Vardecls
         (g-decl (TypedVariable g (TypeChoice LambdaT PutT)))
         (texts-decl (TypedVariable texts ConceptT))
         (ms-decl (TypedVariable ms NumberT))
         ;; Clauses
         (minsup-g (minsup-eval g texts ms)))
  (Bind
    (VariableList
      g-decl
      texts-decl
      ms-decl)
    (And
      minsup-g
      (absolutely-true-eval minsup-g))
    (ExecutionOutput
      (GroundedSchema "scm: shallow-abstraction-formula")
      (List
        (Set)               ; Cannot know the structure of the rule
                            ; conclusion in advance, because we don't
                            ; know the number of shallow abstractions,
                            ; thus we cannot build the Set. Need to
                            ; support ConsLink, or ConsSetLink or
                            ; such.
        minsup-g)))))

;; Shallow abstraction formula
(define (shallow-abstraction-formula conclusion . premises)
  ;; (cog-logger-debug "shallow-abstraction-formula conclusion = ~a, premises = ~a" conclusion premises)
  (if (= (length premises) 1)
      (let* ((minsup-g (car premises))
             (g (get-pattern minsup-g))
             (texts (get-texts minsup-g))
             (ms (get-ms minsup-g))
             (shabs-lists (cog-shallow-abstract g texts ms))
             (list->eval (lambda (x) (cog-set-tv!
                                      (shallow-abstraction-eval x minsup-g)
                                      (stv 1 1))))
             (shabs-evals (map list->eval (cog-outgoing-set shabs-lists))))
        (Set shabs-evals))))

;; Define specialization
(define shallow-abstraction-rule-name
  (DefinedSchemaNode "shallow-abstraction-rule"))
(DefineLink shallow-abstraction-rule-name
  shallow-abstraction-rule)
