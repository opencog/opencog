;; Calculate the I-Surprisingness as defined in
;; https://wiki.opencog.org/w/Measuring_Surprisingness of a pattern
;; composed of the conjunction of components.
;;
;; Semi-formally
;;
;; Evaluation (stv 1 1)
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       <f-vardecl>
;;       And
;;         <cnj-body-1>
;;         ...
;;         <cnj-body-n>
;;     <texts>
;;     <ms>
;; Evaluation (stv 1 1)
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       <cnj-vardecl-1>
;;       <cnj-body-1>
;;     <texts>
;;     <ms>
;; ...
;; Evaluation (stv 1 1)
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       <cnj-vardecl-n>
;;       <cnj-body-n>
;;     <texts>
;;     <ms>
;; |-
;; Evaluation (stv 1 1)
;;   Predicate "I-Surprisingness"
;;   List
;;     Lambda
;;       <f-vardecl>
;;       And
;;        <cnj-body-1>
;;        ...
;;        <cnj-body-n>
;;     <texts>
;;     <isurp>

(load "miner-rule-utils.scm")

;; Generate a rule to calculate the I-Surprisingness of a pattern that
;; is the conjunction of nary components.
;;
;; Assumption: 1 < nary
(define (gen-I-Surprisingness-rule nary)
  ;; Shared variables
  (define f-vardecl (Variable "$f-vardecl"))
  (define texts (Variable "$texts"))
  (define ms (Variable "$ms"))
  ;; Types
  (define VariableT (Type "VariableNode"))
  (define VariableListT (Type "VariableList"))
  (define varT (TypeChoice VariableT VariableListT))
  (define NumberT (Type "NumberNode"))
  (define ConceptT (Type "ConceptNode"))
  ;; Typed declations
  (define typed-f-vardecl (TypedVariable f-vardecl varT))
  (define typed-texts (TypedVariable texts ConceptT))
  (define typed-ms (TypedVariable ms NumberT))

  (if (< 1 nary)
      (let* ((cnj-vardecls (gen-variables "$cnj-vardecls" nary))
             (typed-var (lambda (x) (TypedVariable x varT)))
             (typed-cnj-vardecls (map typed-var cnj-vardecls))
             (cnj-bodies (gen-variables "$cnj-bodies" nary))
             (quoted-lambda (lambda (vardecl body) (Quote
                                                     (Lambda
                                                       (Unquote vardecl)
                                                       (Unquote body)))))
             (cnjs (map quoted-lambda cnj-vardecls cnj-bodies))
             (f (Quote
                  (Lambda
                    (Unquote f-vardecl)
                    (And
                      (map Unquote cnj-bodies)))))
             (f-minsup (minsup-eval f texts ms))
             (cnjs-minsups (map (lambda (x) (minsup-eval x texts ms)) cnjs))
             (f-isurp (isurp-eval f texts)))
        (Bind
          (VariableList
            typed-f-vardecl
            typed-cnj-vardecls
            cnj-bodies
            typed-texts
            typed-ms)
          (And
            (Present
               f-minsup
               cnjs-minsups)
            (Absent
               f-isurp)
            (absolutely-true-eval f-minsup)
            (map absolutely-true-eval cnjs-minsups))
          (ExecutionOutput
            (GroundedSchema "scm: I-Surprisingness-formula")
            (List
              f-isurp
              f-minsup
              (Set cnjs-minsups)))))))

;; I-Suprisingness formula
(define (I-Surprisingness-formula conclusion . premises)
  ;; (cog-logger-debug "(I-Surprisingness-formula conclusion = ~a, premises = ~a"
  ;;                   conclusion premises)

  (if (= 2 (length premises))
      (let* ((pat-isurp conclusion)
             (pat-minsup (car premises))
             (pat (get-pattern pat-minsup))
             (cnjs-minsups (cog-outgoing-set (cadr premises)))
             (cnjs (map get-pattern cnjs-minsups))
             (texts (get-texts (car premises)))

             ;; For now only consider conjuncts independently
             (cnjs-bodies (map get-body cnjs))
             (mk-block (lambda (blk) (if (< 1 (length blk)) (And blk) blk)))
             (mk-partition (lambda (prt) (List (map Lambda (map mk-block prt)))))
             (partitions (List (map mk-partition (cdr (partitions cnjs-bodies)))))

             ;; Calculate I-Surprisingness of pat
             (isurp (cog-I-Surprisingness pat partitions texts)))
        (cog-set-tv! pat-isurp (stv isurp 1)))))

;; Define binary I-Surprisingness
(define I-Surprisingness-2ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-2ary-rule"))
(DefineLink I-Surprisingness-2ary-rule-name
  (gen-I-Surprisingness-rule 2))

;; Define ternary I-Surprisingness
(define I-Surprisingness-3ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-3ary-rule"))
(DefineLink I-Surprisingness-3ary-rule-name
  (gen-I-Surprisingness-rule 3))

;; Define quaternary I-Surprisingness
(define I-Surprisingness-4ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-4ary-rule"))
(DefineLink I-Surprisingness-4ary-rule-name
  (gen-I-Surprisingness-rule 4))

;; Define quinary I-Surprisingness
(define I-Surprisingness-5ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-5ary-rule"))
(DefineLink I-Surprisingness-5ary-rule-name
  (gen-I-Surprisingness-rule 5))

;; Define senary I-Surprisingness
(define I-Surprisingness-6ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-6ary-rule"))
(DefineLink I-Surprisingness-6ary-rule-name
  (gen-I-Surprisingness-rule 6))

;; Define septenary I-Surprisingness
(define I-Surprisingness-7ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-7ary-rule"))
(DefineLink I-Surprisingness-7ary-rule-name
  (gen-I-Surprisingness-rule 7))

;; Define octonary I-Surprisingness
(define I-Surprisingness-8ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-8ary-rule"))
(DefineLink I-Surprisingness-8ary-rule-name
  (gen-I-Surprisingness-rule 8))

;; Define nonary I-Surprisingness
(define I-Surprisingness-9ary-rule-name
  (DefinedSchemaNode "I-Surprisingness-9ary-rule"))
(DefineLink I-Surprisingness-9ary-rule-name
  (gen-I-Surprisingness-rule 9))
