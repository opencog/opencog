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
;; |-
;; Evaluation (stv 1 1)
;;   Predicate "isurp"
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
;;
;; mode can implement variations of I-Surprisingness. Supported variations are
;;
;; 'isurp-old: verbatim port of Shujing I-Surprisingness
;; 'nisurp-old: verbatim port of Shujing normalized I-Surprisingness
;; 'isurp: new implementation of I-Surprisingness (take linkage into account)
;; 'nisurp: new implementation of normalized I-Surprisingness (take linkage into account)
(define (gen-i-surprisingness-rule mode nary)
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
  ;; Formula
  (define formula-name (string-append "scm: " (symbol->string mode) "-formula"))
  (define formula (GroundedSchema formula-name))

  (if (< 1 nary)
      (let* ((cnj-bodies (gen-variables "$cnj-bodies" nary))
             (f (Quote
                  (Lambda
                    (Unquote f-vardecl)
                    (And
                      (map Unquote cnj-bodies)))))
             (f-minsup (minsup-eval f texts ms))
             (f-isurp (isurp-eval f texts)))
        (Bind
          (VariableList
            typed-f-vardecl
            cnj-bodies
            typed-texts
            typed-ms)
          (And
            (Present
               f-minsup)
            (Absent
               f-isurp)
            (absolutely-true-eval f-minsup))
          (ExecutionOutput
            formula
            (List
              f-isurp
              f-minsup))))))

;; I-Suprisingness formula
(define (gen-i-surprisingness-formula mode)
  (lambda (conclusion . premises)

    (cog-logger-debug "(i-surprisingness-formula mode = ~a, conclusion = ~a, premises = ~a"
                      mode conclusion premises)

    (if (= 1 (length premises))
        (let* ((pat-isurp conclusion)
               (pat-minsup (car premises))
               (pat (get-pattern pat-minsup))
               (cnjs-bodies (cog-outgoing-set (get-body pat)))
               (texts (get-texts pat-minsup))

               ;; Calculate I-Surprisingness of pat
               (isurp-op (cond ((equal? mode 'isurp-old) cog-isurp-old)
                               ((equal? mode 'nisurp-old) cog-nisurp-old)
                               ((equal? mode 'isurp) cog-isurp)
                               ((equal? mode 'nisurp) cog-nisurp)))
               (isurp (isurp-op pat texts)))
          (cog-set-tv! pat-isurp (stv isurp 1))))))

;; Old I-Surprisingness

(define isurp-old-formula (gen-i-surprisingness-formula 'isurp-old))
(define (define-isurp-old-rule x)
  (let ((rule-name (string-append "isurp-old-" (number->string x) "ary-rule")))
    (DefineLink
      (DefinedSchemaNode rule-name)
      (gen-i-surprisingness-rule 'isurp-old x))))
(map define-isurp-old-rule (cdr (iota-plus-one 8)))

;; Old normalized I-Surprisingness

(define nisurp-old-formula (gen-i-surprisingness-formula 'nisurp-old))
(define (define-nisurp-old-rule x)
  (let ((rule-name (string-append "nisurp-old-" (number->string x) "ary-rule")))
    (DefineLink
      (DefinedSchemaNode rule-name)
      (gen-i-surprisingness-rule 'nisurp-old x))))
(map define-nisurp-old-rule (cdr (iota-plus-one 8)))

;; New I-Surprisingness

(define isurp-formula (gen-i-surprisingness-formula 'isurp))
(define (define-isurp-rule x)
  (let ((rule-name (string-append "isurp-" (number->string x) "ary-rule")))
    (DefineLink
      (DefinedSchemaNode rule-name)
      (gen-i-surprisingness-rule 'isurp x))))
(map define-isurp-rule (cdr (iota-plus-one 8)))

;; New normalized I-Surprisingness

(define nisurp-formula (gen-i-surprisingness-formula 'nisurp))
(define (define-nisurp-rule x)
  (let ((rule-name (string-append "nisurp-" (number->string x) "ary-rule")))
    (DefineLink
      (DefinedSchemaNode rule-name)
      (gen-i-surprisingness-rule 'nisurp x))))
(map define-nisurp-rule (cdr (iota-plus-one 8)))
